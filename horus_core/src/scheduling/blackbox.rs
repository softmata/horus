//! Black box flight recorder for post-mortem analysis
//!
//! Records all significant events in a circular buffer that persists
//! across crashes for debugging and incident analysis.

use log::info;
use serde::{Deserialize, Serialize};
use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::{BufWriter, Write};
use std::path::PathBuf;
use std::time::{SystemTime, UNIX_EPOCH};

/// Black box event types
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BlackBoxEvent {
    /// Scheduler started
    SchedulerStart {
        name: String,
        node_count: usize,
        config: String,
    },
    /// Scheduler stopped
    SchedulerStop { reason: String, total_ticks: u64 },
    /// Node added
    NodeAdded { name: String, order: u32 },
    /// Node tick completed
    NodeTick {
        name: String,
        duration_us: u64,
        success: bool,
    },
    /// Node error
    NodeError { name: String, error: String },
    /// Deadline miss
    DeadlineMiss {
        name: String,
        deadline_us: u64,
        actual_us: u64,
    },
    /// WCET violation
    WCETViolation {
        name: String,
        budget_us: u64,
        actual_us: u64,
    },
    /// Circuit breaker state change
    CircuitBreakerChange {
        name: String,
        new_state: String,
        failure_count: u32,
    },
    /// Learning phase complete
    LearningComplete {
        duration_ms: u64,
        tier_summary: String,
    },
    /// Emergency stop
    EmergencyStop { reason: String },
    /// Custom event
    Custom { category: String, message: String },
}

/// A recorded event with timestamp
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct BlackBoxRecord {
    /// Timestamp (microseconds since epoch)
    pub timestamp_us: u64,
    /// Monotonic tick counter
    pub tick: u64,
    /// The event
    pub event: BlackBoxEvent,
}

/// Black box recorder with circular buffer
pub struct BlackBox {
    /// Circular buffer of events
    buffer: VecDeque<BlackBoxRecord>,
    /// Maximum buffer size
    max_size: usize,
    /// Current tick counter
    tick_counter: u64,
    /// Path for persistence
    persist_path: Option<PathBuf>,
    /// Write-ahead log file (for crash recovery)
    wal_file: Option<BufWriter<File>>,
    /// Whether recording is enabled
    enabled: bool,
    /// Flush the WAL buffer every N records (default: 64).
    ///
    /// Calling `BufWriter::flush()` on every `record()` invocation turns into
    /// thousands of `fsync`-equivalent syscalls per second at high scheduler
    /// frequencies, stalling the scheduler thread on disk latency.  Batching
    /// the flush amortises the I/O cost at the expense of potentially losing
    /// the last `wal_flush_interval - 1` records on a hard crash.
    wal_flush_interval: usize,
    /// Records written to the WAL since the last flush.
    wal_records_since_flush: usize,
    /// Cumulative count of records silently evicted when the ring buffer was full.
    ///
    /// When the circular buffer reaches `max_size`, the oldest record is
    /// dropped to make room for the new one.  This counter tracks every such
    /// eviction so that post-mortem analysis tools can detect and report gaps
    /// in the event history rather than silently treating a full buffer as a
    /// complete record.
    lost_records: u64,
}

/// Snapshot serialised to and from the `blackbox.json` persistence file.
///
/// Wrapping the event list in a struct lets us store metadata (e.g.
/// `lost_records`) alongside the events in the same JSON document.
/// `load()` falls back to the old bare-array format for backward compatibility.
#[derive(Serialize, Deserialize)]
struct BlackBoxSnapshot {
    lost_records: u64,
    events: Vec<BlackBoxRecord>,
}

impl BlackBox {
    /// Create a new black box recorder
    pub fn new(max_size_mb: usize) -> Self {
        // Estimate ~200 bytes per record
        let max_records = (max_size_mb * 1024 * 1024) / 200;

        Self {
            buffer: VecDeque::with_capacity(max_records.min(100000)),
            max_size: max_records.max(1000),
            tick_counter: 0,
            persist_path: None,
            wal_file: None,
            enabled: max_size_mb > 0,
            wal_flush_interval: 64,
            wal_records_since_flush: 0,
            lost_records: 0,
        }
    }

    /// Set how many WAL records are written between `BufWriter::flush()` calls.
    ///
    /// A higher value reduces I/O syscall frequency at the cost of more
    /// records lost in a hard crash.  The default (64) is a reasonable
    /// balance for RT systems running at 100–1000 Hz:
    ///   - 100 Hz, interval 64 → at most 1 flush per ~640 ms
    ///   - 1000 Hz, interval 64 → at most 1 flush per ~64 ms
    ///
    /// Set to 1 to restore the original per-record fsync behaviour (useful
    /// for debugging or very low-frequency schedulers where the overhead is
    /// acceptable).
    pub fn with_wal_flush_interval(mut self, interval: usize) -> Self {
        self.wal_flush_interval = interval.max(1);
        self
    }

    /// Set persistence path for WAL and JSON snapshot.
    ///
    /// Creates the directory if it doesn't exist. Enables:
    /// - WAL file (`blackbox.wal`) for crash recovery (written on every `record()`)
    /// - JSON snapshot (`blackbox.json`) for clean shutdown (written on `save()`)
    pub fn with_path(mut self, dir: PathBuf) -> Self {
        fs::create_dir_all(&dir).ok();
        let wal_path = dir.join("blackbox.wal");
        if let Ok(file) = fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&wal_path)
        {
            self.wal_file = Some(BufWriter::new(file));
        }
        self.persist_path = Some(dir.join("blackbox.json"));
        self
    }

    /// Record an event
    pub fn record(&mut self, event: BlackBoxEvent) {
        if !self.enabled {
            return;
        }

        let record = BlackBoxRecord {
            timestamp_us: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            tick: self.tick_counter,
            event,
        };

        // Write to WAL first (for crash recovery).
        //
        // BufWriter batches writes in user-space; `flush()` drains the buffer
        // to the OS (fsync-equivalent).  Flushing on every record would cause
        // thousands of syscalls/second at high scheduler frequencies, stalling
        // the RT thread on disk I/O.  Instead we flush every
        // `wal_flush_interval` records; an explicit `flush_wal()` call on
        // scheduler shutdown ensures no records are lost on clean exit.
        if let Some(ref mut wal) = self.wal_file {
            if let Ok(json) = serde_json::to_string(&record) {
                let _ = writeln!(wal, "{}", json);
                self.wal_records_since_flush += 1;
                if self.wal_records_since_flush >= self.wal_flush_interval {
                    let _ = wal.flush();
                    self.wal_records_since_flush = 0;
                }
            }
        }

        // Add to circular buffer, counting every eviction so callers can
        // detect gaps in the event history during post-mortem analysis.
        if self.buffer.len() >= self.max_size {
            self.buffer.pop_front();
            self.lost_records += 1;
        }
        self.buffer.push_back(record);
    }

    /// Flush any buffered WAL data to the OS immediately.
    ///
    /// Must be called on scheduler shutdown to ensure the last batch of
    /// records (up to `wal_flush_interval - 1`) is not lost.  Also safe
    /// to call at any point for a manual sync (e.g., before a checkpoint).
    pub fn flush_wal(&mut self) {
        if let Some(ref mut wal) = self.wal_file {
            let _ = wal.flush();
            self.wal_records_since_flush = 0;
        }
    }

    /// Increment tick counter (call once per scheduler tick)
    pub(crate) fn tick(&mut self) {
        self.tick_counter += 1;
    }

    /// Get all recorded events
    pub fn events(&self) -> Vec<BlackBoxRecord> {
        self.buffer.iter().cloned().collect()
    }

    /// Get all errors and warnings
    pub fn anomalies(&self) -> Vec<BlackBoxRecord> {
        self.buffer
            .iter()
            .filter(|r| {
                matches!(
                    r.event,
                    BlackBoxEvent::NodeError { .. }
                        | BlackBoxEvent::DeadlineMiss { .. }
                        | BlackBoxEvent::WCETViolation { .. }
                        | BlackBoxEvent::EmergencyStop { .. }
                        | BlackBoxEvent::CircuitBreakerChange { .. }
                )
            })
            .cloned()
            .collect()
    }

    /// Save buffer and loss counter to disk.
    ///
    /// The JSON snapshot uses the format:
    /// ```json
    /// { "lost_records": <u64>, "events": [ … ] }
    /// ```
    /// The `lost_records` field allows analysis tools to detect and report
    /// incomplete event history when replaying the snapshot.
    pub fn save(&self) -> std::io::Result<()> {
        if let Some(ref path) = self.persist_path {
            let file = File::create(path)?;
            let writer = BufWriter::new(file);
            let snapshot = BlackBoxSnapshot {
                lost_records: self.lost_records,
                events: self.events(),
            };
            serde_json::to_writer_pretty(writer, &snapshot)?;
            info!(
                "[BLACKBOX] Saved {} events ({} lost) to {:?}",
                self.buffer.len(),
                self.lost_records,
                path
            );
        }
        Ok(())
    }

    /// Load buffer from disk.
    ///
    /// Accepts both the current format (`{ "lost_records": …, "events": […] }`)
    /// and the legacy bare-array format for backward compatibility with snapshots
    /// written before the loss counter was introduced.
    pub fn load(&mut self) -> std::io::Result<()> {
        if let Some(ref path) = self.persist_path {
            if path.exists() {
                let content = fs::read_to_string(path)?;
                // Try new snapshot format first; fall back to bare event array.
                if let Ok(snapshot) = serde_json::from_str::<BlackBoxSnapshot>(&content) {
                    self.lost_records = snapshot.lost_records;
                    self.buffer = VecDeque::from(snapshot.events);
                } else {
                    let events: Vec<BlackBoxRecord> = serde_json::from_str(&content)?;
                    self.buffer = VecDeque::from(events);
                    self.lost_records = 0;
                }
                info!(
                    "[BLACKBOX] Loaded {} events ({} lost) from {:?}",
                    self.buffer.len(),
                    self.lost_records,
                    path
                );
            }
        }
        Ok(())
    }

    /// Clear the buffer and reset counters.
    pub fn clear(&mut self) {
        self.buffer.clear();
        self.tick_counter = 0;
        self.lost_records = 0;
    }

    /// Get buffer size
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }

    /// Return the cumulative count of records lost due to ring buffer overflow.
    ///
    /// A non-zero value means the snapshot is incomplete: at least this many
    /// events were silently evicted before they could be retrieved.  Post-mortem
    /// analysis tools should surface this value prominently when it is non-zero.
    pub fn get_loss_count(&self) -> u64 {
        self.lost_records
    }
}

impl Default for BlackBox {
    fn default() -> Self {
        Self::new(0) // Disabled by default
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_blackbox_record() {
        let mut bb = BlackBox::new(1); // 1MB

        bb.record(BlackBoxEvent::SchedulerStart {
            name: "test".to_string(),
            node_count: 5,
            config: "default".to_string(),
        });

        bb.tick();

        bb.record(BlackBoxEvent::NodeTick {
            name: "sensor".to_string(),
            duration_us: 100,
            success: true,
        });

        assert_eq!(bb.len(), 2);
    }

    #[test]
    fn test_blackbox_circular() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 10; // Override for testing

        for i in 0..20 {
            bb.record(BlackBoxEvent::Custom {
                category: "test".to_string(),
                message: format!("event {}", i),
            });
        }

        // Should only have last 10
        assert_eq!(bb.len(), 10);
    }

    /// WAL flush_interval=64: verify that wal_records_since_flush resets every
    /// 64 records and that an explicit flush_wal() drains any remainder.
    ///
    /// This is a white-box test: we inspect `wal_records_since_flush` directly
    /// rather than mocking the file, which avoids platform-specific I/O setup.
    #[test]
    fn wal_batched_flush_interval() {
        let mut bb = BlackBox::new(1).with_wal_flush_interval(4);
        // No WAL file — flush_wal is a no-op but counter still resets.
        // (We're testing the counter logic, not actual file I/O.)
        assert_eq!(bb.wal_records_since_flush, 0);

        // Write 3 records — should NOT have flushed yet (interval = 4).
        for i in 0..3u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "test".to_string(),
                message: format!("event {}", i),
            });
        }
        assert_eq!(
            bb.wal_records_since_flush, 0,
            "records_since_flush should be 0 when no wal_file is attached"
        );

        // Now attach a real WAL file and re-test with a tmpfile.
        let tmp =
            std::env::temp_dir().join(format!("horus_bb_flush_test_{}.wal", std::process::id()));
        let dir = std::env::temp_dir();
        let mut bb = BlackBox::new(1)
            .with_wal_flush_interval(4)
            .with_path(dir.clone());

        // Write 9 records → 2 full batches (flush at 4, 8) + 1 pending.
        for i in 0..9u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "test".to_string(),
                message: format!("event {}", i),
            });
        }
        // 9 mod 4 = 1 pending record (not yet flushed)
        assert_eq!(
            bb.wal_records_since_flush, 1,
            "should have 1 pending record (9 mod 4)"
        );

        // explicit flush_wal() resets the counter
        bb.flush_wal();
        assert_eq!(
            bb.wal_records_since_flush, 0,
            "flush_wal() must reset counter"
        );

        // Clean up
        let _ = std::fs::remove_file(&tmp);
        let _ = std::fs::remove_file(dir.join("blackbox.wal"));
        let _ = std::fs::remove_file(dir.join("blackbox.json"));
    }

    /// Fill the buffer past capacity and verify the loss counter matches the
    /// number of records that were evicted.
    #[test]
    fn test_loss_counter_increments_on_overflow() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 10;

        // Record 15 events — the first 5 should be evicted.
        for i in 0..15u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "test".to_string(),
                message: format!("event {}", i),
            });
        }

        assert_eq!(bb.len(), 10, "buffer must be capped at max_size");
        assert_eq!(
            bb.get_loss_count(),
            5,
            "5 records were evicted; loss counter must equal the overflow count"
        );
    }

    /// clear() must reset the loss counter along with the buffer.
    #[test]
    fn test_clear_resets_loss_counter() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 3;

        for i in 0..6u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "t".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.get_loss_count(), 3);
        bb.clear();
        assert_eq!(bb.get_loss_count(), 0, "clear() must reset lost_records to 0");
    }

    /// save() must include `lost_records` in the JSON output.
    #[test]
    fn test_save_includes_lost_records() {
        let dir = std::env::temp_dir().join(format!(
            "horus_bb_loss_save_test_{}",
            std::process::id()
        ));
        let mut bb = BlackBox::new(1).with_path(dir.clone());
        bb.max_size = 3;

        for i in 0..6u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "t".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.get_loss_count(), 3);

        bb.save().expect("save must not fail");

        let content = std::fs::read_to_string(dir.join("blackbox.json"))
            .expect("snapshot file must exist");
        let json: serde_json::Value = serde_json::from_str(&content).expect("valid JSON");

        assert_eq!(
            json["lost_records"].as_u64(),
            Some(3),
            "snapshot JSON must contain lost_records = 3"
        );

        // Clean up.
        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn test_blackbox_anomalies() {
        let mut bb = BlackBox::new(1);

        bb.record(BlackBoxEvent::NodeTick {
            name: "ok".to_string(),
            duration_us: 100,
            success: true,
        });

        bb.record(BlackBoxEvent::NodeError {
            name: "bad".to_string(),
            error: "panic".to_string(),
        });

        bb.record(BlackBoxEvent::DeadlineMiss {
            name: "slow".to_string(),
            deadline_us: 1000,
            actual_us: 2000,
        });

        let anomalies = bb.anomalies();
        assert_eq!(anomalies.len(), 2);
    }
}
