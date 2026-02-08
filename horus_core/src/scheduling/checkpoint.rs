//! Checkpoint and restore system for fault tolerance
//!
//! Provides periodic state snapshots that can be used to recover
//! from crashes or rollback to known-good states.

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::PathBuf;
use std::time::{Duration, Instant, SystemTime};

/// Checkpoint manager for periodic state persistence
pub struct CheckpointManager {
    /// Directory to store checkpoints
    checkpoint_dir: PathBuf,
    /// Interval between checkpoints
    interval: Duration,
    /// Last checkpoint time
    last_checkpoint: Instant,
    /// Maximum checkpoints to retain
    max_checkpoints: usize,
    /// Current checkpoint index
    checkpoint_index: u64,
    /// Whether checkpointing is enabled
    enabled: bool,
}

/// A checkpoint containing scheduler state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Checkpoint {
    /// Unique checkpoint ID
    pub id: u64,
    /// Timestamp when created
    pub timestamp_secs: u64,
    /// Node states (serialized)
    pub node_states: HashMap<String, NodeCheckpoint>,
    /// Scheduler metadata
    pub metadata: CheckpointMetadata,
}

/// Checkpoint data for a single node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeCheckpoint {
    /// Node name
    pub name: String,
    /// Total ticks executed
    pub tick_count: u64,
    /// Last tick duration (microseconds)
    pub last_tick_us: u64,
    /// Error count
    pub error_count: u64,
    /// Custom state (serialized by node)
    pub custom_state: Option<Vec<u8>>,
}

/// Metadata about the checkpoint
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct CheckpointMetadata {
    /// Scheduler name
    pub scheduler_name: String,
    /// Total scheduler ticks
    pub total_ticks: u64,
    /// Learning phase complete?
    pub learning_complete: bool,
    /// Number of nodes
    pub node_count: usize,
    /// Uptime in seconds
    pub uptime_secs: f64,
}

impl CheckpointManager {
    /// Create a new checkpoint manager
    pub fn new(checkpoint_dir: PathBuf, interval_ms: u64) -> Self {
        // Create directory if it doesn't exist
        let _ = fs::create_dir_all(&checkpoint_dir);

        Self {
            checkpoint_dir,
            interval: Duration::from_millis(interval_ms),
            last_checkpoint: Instant::now(),
            max_checkpoints: 10,
            checkpoint_index: 0,
            enabled: interval_ms > 0,
        }
    }

    /// Check if it's time to create a checkpoint
    pub fn should_checkpoint(&self) -> bool {
        self.enabled && self.last_checkpoint.elapsed() >= self.interval
    }

    /// Create a checkpoint from current state
    pub fn create_checkpoint(&mut self, metadata: CheckpointMetadata) -> Option<Checkpoint> {
        if !self.enabled {
            return None;
        }

        self.checkpoint_index += 1;
        let checkpoint = Checkpoint {
            id: self.checkpoint_index,
            timestamp_secs: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_secs(),
            node_states: HashMap::new(),
            metadata,
        };

        self.last_checkpoint = Instant::now();
        Some(checkpoint)
    }

    /// Save a checkpoint to disk
    pub fn save_checkpoint(&self, checkpoint: &Checkpoint) -> std::io::Result<PathBuf> {
        let filename = format!("checkpoint_{:08}.bin", checkpoint.id);
        let path = self.checkpoint_dir.join(&filename);

        let file = File::create(&path)?;
        let writer = BufWriter::new(file);
        bincode::serialize_into(writer, checkpoint).map_err(std::io::Error::other)?;

        println!(
            "[CHECKPOINT] Saved checkpoint {} ({} nodes)",
            checkpoint.id, checkpoint.metadata.node_count
        );

        // Cleanup old checkpoints
        self.cleanup_old_checkpoints();

        Ok(path)
    }

    /// Load the latest checkpoint from disk
    pub fn load_latest_checkpoint(&self) -> std::io::Result<Option<Checkpoint>> {
        let mut checkpoints: Vec<_> = fs::read_dir(&self.checkpoint_dir)?
            .filter_map(|e| e.ok())
            .filter(|e| {
                e.path()
                    .file_name()
                    .map(|n| n.to_string_lossy().starts_with("checkpoint_"))
                    .unwrap_or(false)
            })
            .collect();

        if checkpoints.is_empty() {
            return Ok(None);
        }

        // Sort by name (which includes index) descending
        checkpoints.sort_by_key(|b| std::cmp::Reverse(b.path()));

        let latest_path = checkpoints[0].path();
        self.load_checkpoint(&latest_path)
    }

    /// Load a specific checkpoint from disk
    pub fn load_checkpoint(&self, path: &PathBuf) -> std::io::Result<Option<Checkpoint>> {
        if !path.exists() {
            return Ok(None);
        }

        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let checkpoint: Checkpoint =
            bincode::deserialize_from(reader).map_err(std::io::Error::other)?;

        println!(
            "[CHECKPOINT] Loaded checkpoint {} from {:?}",
            checkpoint.id, path
        );

        Ok(Some(checkpoint))
    }

    /// Remove old checkpoints beyond max_checkpoints
    fn cleanup_old_checkpoints(&self) {
        if let Ok(entries) = fs::read_dir(&self.checkpoint_dir) {
            let mut checkpoints: Vec<_> = entries
                .filter_map(|e| e.ok())
                .filter(|e| {
                    e.path()
                        .file_name()
                        .map(|n| n.to_string_lossy().starts_with("checkpoint_"))
                        .unwrap_or(false)
                })
                .collect();

            if checkpoints.len() > self.max_checkpoints {
                // Sort by name ascending (oldest first)
                checkpoints.sort_by_key(|a| a.path());

                // Remove oldest
                let to_remove = checkpoints.len() - self.max_checkpoints;
                for entry in checkpoints.into_iter().take(to_remove) {
                    let _ = fs::remove_file(entry.path());
                }
            }
        }
    }

    /// List all available checkpoints
    pub fn list_checkpoints(&self) -> Vec<(u64, PathBuf)> {
        fs::read_dir(&self.checkpoint_dir)
            .map(|entries| {
                entries
                    .filter_map(|e| e.ok())
                    .filter_map(|e| {
                        let path = e.path();
                        let filename = path.file_name()?.to_string_lossy();
                        if filename.starts_with("checkpoint_") && filename.ends_with(".bin") {
                            let id_str = filename
                                .trim_start_matches("checkpoint_")
                                .trim_end_matches(".bin");
                            let id = id_str.parse::<u64>().ok()?;
                            Some((id, path))
                        } else {
                            None
                        }
                    })
                    .collect()
            })
            .unwrap_or_default()
    }

    /// Set maximum checkpoints to retain
    pub fn set_max_checkpoints(&mut self, max: usize) {
        self.max_checkpoints = max.max(1);
    }

    /// Enable or disable checkpointing
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
}

impl Default for CheckpointManager {
    fn default() -> Self {
        Self::new(
            dirs::cache_dir()
                .unwrap_or_else(|| PathBuf::from("/tmp"))
                .join("horus/checkpoints"),
            0, // Disabled by default
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_checkpoint_save_load() {
        let temp_dir = TempDir::new().unwrap();
        let mut manager = CheckpointManager::new(temp_dir.path().to_path_buf(), 100);

        let metadata = CheckpointMetadata {
            scheduler_name: "test".to_string(),
            total_ticks: 1000,
            learning_complete: true,
            node_count: 5,
            uptime_secs: 60.0,
        };

        let checkpoint = manager.create_checkpoint(metadata).unwrap();
        let path = manager.save_checkpoint(&checkpoint).unwrap();

        let loaded = manager
            .load_checkpoint(&path)
            .expect("load failed")
            .expect("checkpoint not found");
        assert_eq!(loaded.id, checkpoint.id);
        assert_eq!(loaded.metadata.scheduler_name, "test");
    }

    #[test]
    fn test_checkpoint_cleanup() {
        let temp_dir = TempDir::new().unwrap();
        let mut manager = CheckpointManager::new(temp_dir.path().to_path_buf(), 1);
        manager.set_max_checkpoints(3);

        // Create more checkpoints than max
        for i in 0..5 {
            let metadata = CheckpointMetadata {
                scheduler_name: "test".to_string(),
                total_ticks: i * 100,
                learning_complete: false,
                node_count: 1,
                uptime_secs: i as f64,
            };
            let checkpoint = manager.create_checkpoint(metadata).unwrap();
            manager.save_checkpoint(&checkpoint).unwrap();
        }

        // Should only have max_checkpoints remaining
        let checkpoints = manager.list_checkpoints();
        assert!(checkpoints.len() <= 3);
    }
}
