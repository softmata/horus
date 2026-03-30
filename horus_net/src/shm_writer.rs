//! ShmRingWriter — write incoming network data to local SHM topics.
//!
//! Remote data arrives as raw bytes from the network. ShmRingWriter writes it
//! into the local SHM ring buffer so local subscribers read it via Topic<T>.recv().
//!
//! This directly writes to the memory-mapped SHM file using the same layout
//! as horus_core's Topic<T>.send(). It's a "virtual publisher" that feeds
//! network-received data into the local topic system.

use std::fs::{File, OpenOptions};
use std::io;
use std::path::PathBuf;
use std::sync::atomic::Ordering;

use memmap2::MmapMut;

use horus_sys::shm::shm_topics_dir;

use crate::priority::Encoding;

/// Header constants (must match horus_core::communication::topic::header).
const TOPIC_MAGIC: u64 = 0x4144415054495645; // "ADAPTIVE"
const TOPIC_HEADER_SIZE: usize = 640;

// Field offsets within TopicHeader (from header.rs layout analysis):
const OFF_MAGIC: usize = 0; // u64
const OFF_TYPE_SIZE: usize = 12; // u32
const OFF_IS_POD: usize = 20; // u8 (1=no, 2=yes)
const OFF_HEAD: usize = 64; // u64 (sequence_or_head)
const OFF_CAPACITY: usize = 72; // u32
const OFF_CAP_MASK: usize = 76; // u32
const OFF_SLOT_SIZE: usize = 80; // u32
const OFF_MSG_TOTAL: usize = 48; // u64 (messages_total)

const POD_YES: u8 = 2;

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
    /// The SHM file must already exist (created by a local subscriber's Topic<T>::new()).
    /// Returns `None` if the file doesn't exist or the header is invalid.
    pub fn open(topic_name: &str) -> Option<Self> {
        let sanitized = topic_name.replace('.', "_").replace('/', "_");
        let path = shm_topics_dir().join(format!("horus_{sanitized}"));
        Self::open_path(topic_name, &path)
    }

    /// Open with a specific path (for testing).
    pub fn open_path(topic_name: &str, path: &std::path::Path) -> Option<Self> {
        let file = OpenOptions::new().read(true).write(true).open(path).ok()?;
        let meta = file.metadata().ok()?;
        if (meta.len() as usize) < TOPIC_HEADER_SIZE {
            return None;
        }

        // SAFETY: file is opened read-write, we validated minimum size.
        let mmap = unsafe { MmapMut::map_mut(&file).ok()? };

        // Validate magic
        let magic = u64::from_ne_bytes(mmap[OFF_MAGIC..OFF_MAGIC + 8].try_into().ok()?);
        if magic != TOPIC_MAGIC {
            return None;
        }

        // Read stable header fields
        let type_size =
            u32::from_ne_bytes(mmap[OFF_TYPE_SIZE..OFF_TYPE_SIZE + 4].try_into().ok()?) as usize;
        let is_pod = mmap[OFF_IS_POD] == POD_YES;
        let capacity =
            u32::from_ne_bytes(mmap[OFF_CAPACITY..OFF_CAPACITY + 4].try_into().ok()?) as usize;
        let cap_mask =
            u32::from_ne_bytes(mmap[OFF_CAP_MASK..OFF_CAP_MASK + 4].try_into().ok()?) as usize;
        let slot_size =
            u32::from_ne_bytes(mmap[OFF_SLOT_SIZE..OFF_SLOT_SIZE + 4].try_into().ok()?) as usize;

        if capacity == 0 {
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

    /// Write POD payload directly to the ring slot.
    fn write_pod(&mut self, payload: &[u8]) -> bool {
        if payload.len() != self.type_size {
            return false; // Size mismatch
        }

        // Read current head
        let head_ptr = &self.mmap[OFF_HEAD..OFF_HEAD + 8];
        let head = u64::from_ne_bytes(head_ptr.try_into().unwrap());

        // Calculate slot position
        let slot_idx = (head as usize) & self.cap_mask;
        let slot_start = TOPIC_HEADER_SIZE + slot_idx * self.type_size;
        let slot_end = slot_start + self.type_size;

        if slot_end > self.mmap.len() {
            return false; // SHM file too small
        }

        // Write payload to slot
        self.mmap[slot_start..slot_end].copy_from_slice(payload);

        // Advance head atomically (store with Release ordering for visibility)
        let new_head = head.wrapping_add(1);
        self.mmap[OFF_HEAD..OFF_HEAD + 8].copy_from_slice(&new_head.to_ne_bytes());

        // Increment messages_total
        let total_ptr = &self.mmap[OFF_MSG_TOTAL..OFF_MSG_TOTAL + 8];
        let total = u64::from_ne_bytes(total_ptr.try_into().unwrap());
        self.mmap[OFF_MSG_TOTAL..OFF_MSG_TOTAL + 8]
            .copy_from_slice(&(total.wrapping_add(1)).to_ne_bytes());

        true
    }

    /// Write serialized payload to the ring slot (with length prefix).
    fn write_serialized(&mut self, payload: &[u8]) -> bool {
        if self.slot_size < 16 {
            return false;
        }
        let max_payload = self.slot_size - 16; // Subtract slot header (8B len + 8B padding)
        if payload.len() > max_payload {
            return false; // Payload too large for slot
        }

        // Read current head
        let head_ptr = &self.mmap[OFF_HEAD..OFF_HEAD + 8];
        let head = u64::from_ne_bytes(head_ptr.try_into().unwrap());

        // Calculate slot position
        let slot_idx = (head as usize) & self.cap_mask;
        let slot_start = TOPIC_HEADER_SIZE + slot_idx * self.slot_size;

        if slot_start + self.slot_size > self.mmap.len() {
            return false;
        }

        // Write length prefix (8 bytes, little-endian) + payload
        let len_bytes = (payload.len() as u64).to_le_bytes();
        self.mmap[slot_start..slot_start + 8].copy_from_slice(&len_bytes);
        // 8 bytes padding/reserved
        self.mmap[slot_start + 8..slot_start + 16].copy_from_slice(&[0u8; 8]);
        // Payload
        self.mmap[slot_start + 16..slot_start + 16 + payload.len()].copy_from_slice(payload);

        // Advance head
        let new_head = head.wrapping_add(1);
        self.mmap[OFF_HEAD..OFF_HEAD + 8].copy_from_slice(&new_head.to_ne_bytes());

        // Increment messages_total
        let total_ptr = &self.mmap[OFF_MSG_TOTAL..OFF_MSG_TOTAL + 8];
        let total = u64::from_ne_bytes(total_ptr.try_into().unwrap());
        self.mmap[OFF_MSG_TOTAL..OFF_MSG_TOTAL + 8]
            .copy_from_slice(&(total.wrapping_add(1)).to_ne_bytes());

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn writer_nonexistent_file_returns_none() {
        let result = ShmRingWriter::open_path(
            "ghost",
            std::path::Path::new("/tmp/nonexistent_horus_net_shm_writer_test"),
        );
        assert!(result.is_none());
    }

    // Full integration tests (writer + reader) require a real SHM topic.
    // These are tested in the R3 E2E test suite where we create actual topics.
}
