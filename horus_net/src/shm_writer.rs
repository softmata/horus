//! ShmRingWriter — write incoming network data to local SHM topics.
//!
//! Remote data arrives as raw bytes from the network. ShmRingWriter writes it
//! into the local SHM ring buffer so local subscribers read it via `Topic<T>::recv()`.
//!
//! This directly writes to the memory-mapped SHM file using the same layout
//! as horus_core's `Topic<T>::send()`. It's a "virtual publisher" that feeds
//! network-received data into the local topic system.
//!
//! # Layout is not duplicated here — deliberately
//!
//! This module used to hard-code a table of byte offsets copied out of
//! `horus_core`'s `header.rs`. The copy drifted, and because the file path was
//! *also* wrong (below) nothing ever exercised it, so the drift went unnoticed
//! from 2026-03-29 until the 2026-07-30 security audit. All four copied facts
//! were wrong by then: the data region omitted the sequence array, the
//! `messages_total` offset actually pointed at `topic_kind`, the serde length
//! field was written where the ready word lives, and the per-slot ready flag was
//! never published at all.
//!
//! Since the writer is fed by **unauthenticated network data**, each of those is
//! remote-controlled corruption of a live IPC control structure the moment the
//! path bug is fixed. Every offset now comes from
//! [`horus_core::communication::topic::shm_layout`], whose constants are
//! `offset_of!`-asserted against the real `TopicHeader`, so the same drift is now
//! a compile error.
//!
//! Likewise the backing-file path comes from [`horus_sys::shm::topic_shm_path`],
//! the single definition also used by `ShmRegion::new`. This module previously
//! looked for `horus_<sanitized-name>` while `horus_core` had stopped writing the
//! `horus_` prefix — so `open()` returned `None` for every real topic and LAN
//! replication was silently dead in both directions.

use std::fs::OpenOptions;
use std::sync::atomic::{fence, AtomicU64, Ordering};

use memmap2::MmapMut;

use horus_core::communication::topic::shm_layout as layout;

use crate::priority::Encoding;

/// Writer that pushes network-received data into a local SHM ring buffer.
pub struct ShmRingWriter {
    /// Memory-mapped SHM file (read-write).
    mmap: MmapMut,
    /// Cached header values (read once, stable after creation).
    type_size: usize,
    capacity: usize,
    cap_mask: usize,
    slot_size: usize,
    is_pod: bool,
    /// Topic name for logging.
    topic_name: String,
}

impl ShmRingWriter {
    /// Open an existing SHM topic file for writing.
    ///
    /// The SHM file must already exist (created by a local subscriber's `Topic<T>::new()`).
    /// Returns `None` if the file doesn't exist or the header is invalid.
    pub fn open(topic_name: &str) -> Option<Self> {
        let path = horus_sys::shm::topic_shm_path(topic_name);
        Self::open_path(topic_name, &path)
    }

    /// Open with a specific path (for testing).
    pub fn open_path(topic_name: &str, path: &std::path::Path) -> Option<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path).ok()?;
        let meta = file.metadata().ok()?;
        if (meta.len() as usize) < layout::HEADER_SIZE {
            return None;
        }

        // SAFETY: file is opened read-write, we validated minimum size.
        let mmap = unsafe { MmapMut::map_mut(&file).ok()? };

        // Validate magic
        let magic = u64::from_ne_bytes(read_at(&mmap, layout::OFF_MAGIC)?);
        if magic != layout::MAGIC {
            return None;
        }

        // Read stable header fields
        let type_size = u32::from_ne_bytes(read_at(&mmap, layout::OFF_TYPE_SIZE)?) as usize;
        let is_pod = mmap[layout::OFF_IS_POD] == layout::IS_POD_YES;
        let capacity = u32::from_ne_bytes(read_at(&mmap, layout::OFF_CAPACITY)?) as usize;
        let cap_mask = u32::from_ne_bytes(read_at(&mmap, layout::OFF_CAPACITY_MASK)?) as usize;
        let slot_size = u32::from_ne_bytes(read_at(&mmap, layout::OFF_SLOT_SIZE)?) as usize;

        if capacity == 0 {
            return None;
        }
        // A non-power-of-two capacity would make `head & cap_mask` land outside
        // [0, capacity), so reject it here rather than bounds-check every write.
        if !capacity.is_power_of_two() || cap_mask != capacity - 1 {
            return None;
        }

        // The whole ring must be inside the mapping. Checking once here means the
        // per-message path cannot compute an in-bounds-looking offset from a
        // truncated file.
        let stride = if is_pod { type_size } else { slot_size };
        if stride == 0 || mmap.len() < layout::required_region_len(capacity, stride) {
            return None;
        }

        Some(Self {
            mmap,
            type_size,
            capacity,
            cap_mask,
            slot_size,
            is_pod,
            topic_name: topic_name.to_string(),
        })
    }

    /// Write raw bytes into the ring buffer.
    ///
    /// For POD data: `payload` is raw struct bytes, written directly to the slot.
    /// For serialized data: `payload` is bincode bytes, written with length prefix.
    ///
    /// This advances the head atomically so local subscribers see the new data.
    pub fn write(&mut self, payload: &[u8], _encoding: Encoding) -> bool {
        if self.is_pod {
            self.write_pod(payload)
        } else {
            self.write_serialized(payload)
        }
    }

    /// Load the producer publish counter.
    fn load_head(&self) -> u64 {
        // SAFETY: OFF_SEQUENCE_OR_HEAD is inside the header, which `open_path`
        // proved is mapped, and the field is 8-byte aligned within a 64-byte
        // aligned mapping. horus_core accesses the same word as an AtomicU64, so
        // it must be accessed atomically here too — a plain read would be a data
        // race, which is what this module used to do.
        unsafe {
            let ptr = self.mmap.as_ptr().add(layout::OFF_SEQUENCE_OR_HEAD) as *const AtomicU64;
            (*ptr).load(Ordering::Acquire)
        }
    }

    /// Publish `new_head`, making the slot visible to consumers.
    fn publish_head(&mut self, new_head: u64) {
        // SAFETY: see `load_head`. Release pairs with the consumer's Acquire load
        // of the same word, ordering the payload write before its visibility.
        unsafe {
            let ptr = self.mmap.as_ptr().add(layout::OFF_SEQUENCE_OR_HEAD) as *const AtomicU64;
            (*ptr).store(new_head, Ordering::Release);
        }
    }

    /// Publish the per-slot ready word at `offset`.
    ///
    /// Consumers under the multi-producer backend gate on this, not on the head.
    /// Omitting it — as this module used to — means remote data is written and
    /// then never delivered on any topic that has a local publisher too.
    fn publish_ready(&mut self, offset: usize, value: u64) {
        debug_assert!(offset + 8 <= self.mmap.len());
        // SAFETY: `offset` is derived from `seq_slot_offset`/slot base with
        // `index < capacity`, and `open_path` validated the whole ring is mapped.
        // Both are 8-byte aligned relative to a page-aligned mapping.
        unsafe {
            let ptr = self.mmap.as_ptr().add(offset) as *const AtomicU64;
            (*ptr).store(value, Ordering::Release);
        }
    }

    /// Bump `messages_total`, the always-on send counter.
    fn bump_messages_total(&mut self) {
        // SAFETY: OFF_MESSAGES_TOTAL is inside the validated header and 8-byte
        // aligned. Accessed atomically because horus_core declares it AtomicU64.
        unsafe {
            let ptr = self.mmap.as_ptr().add(layout::OFF_MESSAGES_TOTAL) as *const AtomicU64;
            (*ptr).fetch_add(1, Ordering::Relaxed);
        }
    }

    /// Write POD payload directly to the ring slot.
    fn write_pod(&mut self, payload: &[u8]) -> bool {
        if payload.len() != self.type_size {
            return false; // Size mismatch
        }

        let head = self.load_head();
        let index = (head as usize) & self.cap_mask;
        let slot_start = layout::data_slot_offset(self.capacity, index, self.type_size);
        let slot_end = slot_start + self.type_size;

        if slot_end > self.mmap.len() {
            return false; // SHM file too small (should be unreachable after open_path)
        }

        self.mmap[slot_start..slot_end].copy_from_slice(payload);

        // Order the payload write ahead of both publish points.
        fence(Ordering::Release);

        let new_head = head.wrapping_add(1);
        // POD readiness lives in the sequence array; publish there as well as on
        // the head so the data survives an SpscShm -> MpscShm migration, matching
        // what dispatch::send_shm_sp_pod does.
        self.publish_ready(layout::seq_slot_offset(index), new_head);
        self.publish_head(new_head);
        self.bump_messages_total();

        true
    }

    /// Write serialized payload to the ring slot (with length prefix).
    fn write_serialized(&mut self, payload: &[u8]) -> bool {
        if self.slot_size < layout::SERDE_SLOT_OVERHEAD {
            return false;
        }
        let max_payload = self.slot_size - layout::SERDE_SLOT_OVERHEAD;
        if payload.len() > max_payload {
            return false; // Payload too large for slot
        }

        let head = self.load_head();
        let index = (head as usize) & self.cap_mask;
        let slot_start = layout::data_slot_offset(self.capacity, index, self.slot_size);

        if slot_start + self.slot_size > self.mmap.len() {
            return false; // should be unreachable after open_path
        }

        // Serde slot layout is [8B ready | 8B length | data...]. The length goes
        // at +8, NOT at +0: +0 is the ready word, and writing the length there
        // both loses the length and forges a readiness value.
        let len_off = slot_start + layout::SERDE_SLOT_LEN_OFF;
        let data_off = slot_start + layout::SERDE_SLOT_DATA_OFF;
        self.mmap[len_off..len_off + 8].copy_from_slice(&(payload.len() as u64).to_le_bytes());
        self.mmap[data_off..data_off + payload.len()].copy_from_slice(payload);

        fence(Ordering::Release);

        let new_head = head.wrapping_add(1);
        // Serde readiness is the first word of the slot itself.
        self.publish_ready(slot_start + layout::SERDE_SLOT_READY_OFF, new_head);
        self.publish_head(new_head);
        self.bump_messages_total();

        true
    }

    /// Topic name.
    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }

    /// Whether this topic uses POD types.
    pub fn is_pod(&self) -> bool {
        self.is_pod
    }
}

/// Read a fixed-width little-endian-agnostic field, returning `None` if the
/// mapping is too short rather than panicking on a truncated or hostile file.
fn read_at<const N: usize>(mmap: &MmapMut, offset: usize) -> Option<[u8; N]> {
    mmap.get(offset..offset + N)?.try_into().ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    #[test]
    fn writer_nonexistent_file_returns_none() {
        let result = ShmRingWriter::open_path(
            "ghost",
            std::path::Path::new("/tmp/nonexistent_horus_net_shm_writer_test"),
        );
        assert!(result.is_none());
    }

    /// Build a minimal but valid POD topic region on disk.
    fn make_pod_region(path: &std::path::Path, capacity: usize, type_size: usize) {
        let total = layout::required_region_len(capacity, type_size);
        let mut buf = vec![0u8; total];
        buf[layout::OFF_MAGIC..layout::OFF_MAGIC + 8].copy_from_slice(&layout::MAGIC.to_ne_bytes());
        buf[layout::OFF_TYPE_SIZE..layout::OFF_TYPE_SIZE + 4]
            .copy_from_slice(&(type_size as u32).to_ne_bytes());
        buf[layout::OFF_IS_POD] = layout::IS_POD_YES;
        buf[layout::OFF_CAPACITY..layout::OFF_CAPACITY + 4]
            .copy_from_slice(&(capacity as u32).to_ne_bytes());
        buf[layout::OFF_CAPACITY_MASK..layout::OFF_CAPACITY_MASK + 4]
            .copy_from_slice(&((capacity - 1) as u32).to_ne_bytes());
        buf[layout::OFF_SLOT_SIZE..layout::OFF_SLOT_SIZE + 4]
            .copy_from_slice(&(type_size as u32).to_ne_bytes());
        let mut f = std::fs::File::create(path).unwrap();
        f.write_all(&buf).unwrap();
    }

    fn tmp(name: &str) -> std::path::PathBuf {
        std::env::temp_dir().join(format!("horus_shmw_{}_{}", name, std::process::id()))
    }

    #[test]
    fn pod_write_lands_in_the_data_region_not_the_seq_array() {
        // The core regression: with capacity 8 the data region starts 64 bytes
        // after the header. A writer that forgets the sequence array puts the
        // payload on top of the ready flags instead.
        let path = tmp("pod_offset");
        make_pod_region(&path, 8, 4);
        let mut w = ShmRingWriter::open_path("t", &path).expect("region must open");
        assert!(w.write(&[0xAA, 0xBB, 0xCC, 0xDD], Encoding::PodLe));

        let bytes = std::fs::read(&path).unwrap();
        let data_start = layout::data_region_offset(8);
        assert_eq!(
            &bytes[data_start..data_start + 4],
            &[0xAA, 0xBB, 0xCC, 0xDD],
            "payload must be at data_region_offset, not HEADER_SIZE"
        );
        assert_ne!(
            &bytes[layout::HEADER_SIZE..layout::HEADER_SIZE + 4],
            &[0xAA, 0xBB, 0xCC, 0xDD],
            "payload must NOT be sitting on the sequence array"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn pod_write_publishes_head_and_slot_ready_flag() {
        let path = tmp("pod_ready");
        make_pod_region(&path, 4, 8);
        let mut w = ShmRingWriter::open_path("t", &path).expect("region must open");
        assert!(w.write(&[1, 2, 3, 4, 5, 6, 7, 8], Encoding::PodLe));

        let bytes = std::fs::read(&path).unwrap();
        let head = u64::from_ne_bytes(
            bytes[layout::OFF_SEQUENCE_OR_HEAD..layout::OFF_SEQUENCE_OR_HEAD + 8]
                .try_into()
                .unwrap(),
        );
        assert_eq!(head, 1);
        let ready = u64::from_ne_bytes(
            bytes[layout::seq_slot_offset(0)..layout::seq_slot_offset(0) + 8]
                .try_into()
                .unwrap(),
        );
        assert_eq!(ready, 1, "MpscShm consumers gate on this word");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn pod_write_increments_messages_total_and_leaves_topic_kind_alone() {
        let path = tmp("pod_counter");
        make_pod_region(&path, 4, 8);
        let mut w = ShmRingWriter::open_path("t", &path).expect("region must open");
        for _ in 0..3 {
            assert!(w.write(&[0; 8], Encoding::PodLe));
        }

        let bytes = std::fs::read(&path).unwrap();
        let total = u64::from_ne_bytes(
            bytes[layout::OFF_MESSAGES_TOTAL..layout::OFF_MESSAGES_TOTAL + 8]
                .try_into()
                .unwrap(),
        );
        assert_eq!(total, 3);
        assert_eq!(
            bytes[layout::OFF_TOPIC_KIND],
            0,
            "topic_kind must be untouched — the old writer clobbered it with the counter"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn pod_write_wraps_within_the_ring() {
        let capacity = 4;
        let path = tmp("pod_wrap");
        make_pod_region(&path, capacity, 4);
        let mut w = ShmRingWriter::open_path("t", &path).expect("region must open");
        // Write capacity+1 messages; the last must land back in slot 0.
        for i in 0..=capacity {
            assert!(w.write(&[i as u8; 4], Encoding::PodLe));
        }
        let bytes = std::fs::read(&path).unwrap();
        let slot0 = layout::data_slot_offset(capacity, 0, 4);
        assert_eq!(&bytes[slot0..slot0 + 4], &[capacity as u8; 4]);
        // Nothing may have been written past the end of the region.
        assert_eq!(bytes.len(), layout::required_region_len(capacity, 4));
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn pod_write_rejects_wrong_size_payload() {
        let path = tmp("pod_size");
        make_pod_region(&path, 4, 8);
        let mut w = ShmRingWriter::open_path("t", &path).expect("region must open");
        // A remote peer can send any payload length for a registered topic.
        assert!(!w.write(&[0; 7], Encoding::PodLe), "short payload");
        assert!(!w.write(&[0; 9], Encoding::PodLe), "long payload");
        assert!(!w.write(&[], Encoding::PodLe), "empty payload");
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn open_rejects_non_power_of_two_capacity() {
        // cap_mask would not partition the ring, so `head & mask` could exceed
        // capacity and index past the data region.
        let path = tmp("bad_cap");
        make_pod_region(&path, 8, 4);
        let mut bytes = std::fs::read(&path).unwrap();
        bytes[layout::OFF_CAPACITY..layout::OFF_CAPACITY + 4].copy_from_slice(&7u32.to_ne_bytes());
        std::fs::write(&path, &bytes).unwrap();
        assert!(ShmRingWriter::open_path("t", &path).is_none());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn open_rejects_region_truncated_below_its_declared_ring() {
        let path = tmp("truncated");
        make_pod_region(&path, 8, 16);
        let bytes = std::fs::read(&path).unwrap();
        // Keep the header (so magic/geometry still parse) but drop most slots.
        std::fs::write(&path, &bytes[..layout::HEADER_SIZE + 32]).unwrap();
        assert!(
            ShmRingWriter::open_path("t", &path).is_none(),
            "a file smaller than its declared geometry must be refused, not indexed into"
        );
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn open_rejects_bad_magic() {
        let path = tmp("bad_magic");
        make_pod_region(&path, 4, 4);
        let mut bytes = std::fs::read(&path).unwrap();
        bytes[0] ^= 0xFF;
        std::fs::write(&path, &bytes).unwrap();
        assert!(ShmRingWriter::open_path("t", &path).is_none());
        let _ = std::fs::remove_file(&path);
    }

    #[test]
    fn open_uses_the_canonical_topic_path() {
        // Regression guard for the four-month seam break: the writer must look
        // exactly where ShmRegion::new writes, with no added prefix.
        let expected = horus_sys::shm::topic_shm_path("robot.imu");
        assert!(
            expected.ends_with("robot.imu"),
            "path must be the raw topic name, got {}",
            expected.display()
        );
        assert!(
            !expected.to_string_lossy().contains("horus_robot"),
            "the legacy horus_ prefix must not be reintroduced"
        );
    }
}
