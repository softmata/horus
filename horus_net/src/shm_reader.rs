//! ShmRingReader — read new data from local SHM topics for network export.
//!
//! Wraps `horus_core::communication::topic::read_latest_slot_bytes` to poll
//! local SHM ring buffers and yield new entries since last read.

use std::path::PathBuf;

use horus_core::communication::topic::read_latest_slot_bytes;
use horus_sys::shm::topic_shm_path;

use crate::priority::Encoding;

/// Raw message read from SHM, ready for network transmission.
#[derive(Debug, Clone)]
pub struct RawShmMessage {
    /// Raw payload bytes (POD struct bytes or bincode-serialized).
    pub data: Vec<u8>,
    /// Monotonic write index (serves as sequence number).
    pub write_idx: u64,
    /// Whether the payload is POD (raw bytes) or serialized.
    pub is_pod: bool,
    /// Encoding tag for the wire format.
    pub encoding: Encoding,
}

/// Reader that polls a local SHM topic ring buffer for new data.
///
/// Each reader tracks its last-read position. Calling `try_read_next()`
/// returns new data or `None` if no new messages since last read.
pub struct ShmRingReader {
    /// Path to the SHM backing file for this topic.
    shm_path: PathBuf,
    /// Last write index we've seen — used to detect new data.
    last_write_idx: u64,
    /// Topic name (for logging).
    topic_name: String,
}

impl ShmRingReader {
    /// Create a reader for a specific topic.
    pub fn new(topic_name: &str) -> Self {
        // The path must come from the same helper `ShmRegion::new` uses. This
        // previously built `horus_<sanitized>`, which stopped matching when the
        // `horus_` prefix was dropped on 2026-03-29 — so the export side of LAN
        // replication silently read nothing for four months.
        let shm_path = topic_shm_path(topic_name);

        Self {
            shm_path,
            last_write_idx: 0,
            topic_name: topic_name.to_string(),
        }
    }

    /// Create a reader with a custom SHM path (for testing).
    pub fn with_path(topic_name: &str, shm_path: PathBuf) -> Self {
        Self {
            shm_path,
            last_write_idx: 0,
            topic_name: topic_name.to_string(),
        }
    }

    /// Try to read the latest message from the SHM ring buffer.
    ///
    /// Returns `Some(RawShmMessage)` if new data is available since the last read.
    /// Returns `None` if no new data, or if the SHM file doesn't exist yet.
    ///
    /// This is non-blocking and allocation-free on the "no new data" path.
    pub fn try_read_latest(&mut self) -> Option<RawShmMessage> {
        let slot = read_latest_slot_bytes(&self.shm_path, self.last_write_idx)?;

        // Update tracking
        let gap = if self.last_write_idx > 0 {
            slot.write_idx.saturating_sub(self.last_write_idx)
        } else {
            1
        };
        if gap > 1 {
            // We missed some messages (ring lapped us or we're slow).
            // Only the latest is available — this is fine for network replication
            // since we always want the freshest data.
        }
        self.last_write_idx = slot.write_idx;

        let encoding = if slot.is_pod {
            Encoding::native_pod()
        } else {
            Encoding::Bincode
        };

        Some(RawShmMessage {
            data: slot.payload,
            write_idx: slot.write_idx,
            is_pod: slot.is_pod,
            encoding,
        })
    }

    /// Check if the SHM file exists (topic has been created locally).
    pub fn shm_exists(&self) -> bool {
        self.shm_path.exists()
    }

    /// Topic name.
    pub fn topic_name(&self) -> &str {
        &self.topic_name
    }

    /// Current write index (0 if never read).
    pub fn last_write_idx(&self) -> u64 {
        self.last_write_idx
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reader_creation() {
        let reader = ShmRingReader::new("robot.imu");
        assert_eq!(reader.topic_name(), "robot.imu");
        assert_eq!(reader.last_write_idx(), 0);
        // SHM file won't exist in test — that's fine
    }

    #[test]
    fn reader_nonexistent_shm_returns_none() {
        let mut reader = ShmRingReader::with_path(
            "ghost_topic",
            PathBuf::from("/tmp/nonexistent_shm_file_horus_net_test"),
        );
        assert!(reader.try_read_latest().is_none());
    }

    #[test]
    fn reader_path_matches_where_horus_core_actually_writes() {
        // This test previously asserted the mangled `horus_robot_front_lidar_scan`
        // form, which is how the export seam stayed broken for four months with a
        // green suite: the test encoded the bug rather than the contract. The
        // contract is `ShmRegion::new`'s path, and nothing else.
        let name = "robot.front_lidar.scan";
        let reader = ShmRingReader::new(name);
        assert_eq!(
            reader.shm_path,
            topic_shm_path(name),
            "reader must open exactly the file ShmRegion::new creates"
        );
        assert!(
            reader.shm_path.ends_with(name),
            "the topic name is used verbatim — no prefix, no character mangling"
        );
    }

    #[test]
    fn reader_and_writer_agree_on_the_path() {
        // The two halves of the replication seam diverging is the original bug;
        // pin them to each other so they cannot drift apart again silently.
        let name = "robot.imu";
        let reader = ShmRingReader::new(name);
        assert_eq!(reader.shm_path, topic_shm_path(name));
    }
}
