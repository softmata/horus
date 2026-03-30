//! ShmRingReader — read new data from local SHM topics for network export.
//!
//! Wraps `horus_core::communication::topic::read_latest_slot_bytes` to poll
//! local SHM ring buffers and yield new entries since last read.

use std::path::PathBuf;

use horus_core::communication::topic::{read_latest_slot_bytes, TopicSlotRead};
use horus_sys::shm::shm_topics_dir;

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
        // SHM file naming: shm_topics_dir()/horus_{sanitized_name}
        let sanitized = topic_name.replace('.', "_").replace('/', "_");
        let shm_path = shm_topics_dir().join(format!("horus_{sanitized}"));

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
    fn shm_path_sanitization() {
        let reader = ShmRingReader::new("robot.front_lidar.scan");
        let expected_suffix = "horus_robot_front_lidar_scan";
        assert!(
            reader.shm_path.to_str().unwrap().ends_with(expected_suffix),
            "path {:?} should end with {expected_suffix}",
            reader.shm_path
        );
    }
}
