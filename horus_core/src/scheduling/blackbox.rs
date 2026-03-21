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
    /// Node error (includes severity for post-mortem triage)
    NodeError {
        name: String,
        error: String,
        severity: crate::error::Severity,
    },
    /// Deadline miss
    DeadlineMiss {
        name: String,
        deadline_us: u64,
        actual_us: u64,
    },
    /// budget violation
    BudgetViolation {
        name: String,
        budget_us: u64,
        actual_us: u64,
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
                        | BlackBoxEvent::BudgetViolation { .. }
                        | BlackBoxEvent::EmergencyStop { .. }
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
        assert_eq!(
            bb.get_loss_count(),
            0,
            "clear() must reset lost_records to 0"
        );
    }

    /// save() must include `lost_records` in the JSON output.
    #[test]
    fn test_save_includes_lost_records() {
        let dir =
            std::env::temp_dir().join(format!("horus_bb_loss_save_test_{}", std::process::id()));
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

        let content =
            std::fs::read_to_string(dir.join("blackbox.json")).expect("snapshot file must exist");
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
            severity: crate::error::Severity::Fatal,
        });

        bb.record(BlackBoxEvent::DeadlineMiss {
            name: "slow".to_string(),
            deadline_us: 1000,
            actual_us: 2000,
        });

        let anomalies = bb.anomalies();
        assert_eq!(anomalies.len(), 2);
    }

    // ========================================================================
    // Blackbox hardening tests
    // ========================================================================

    #[test]
    fn test_blackbox_many_records_no_crash() {
        let mut bb = BlackBox::new(1); // 1 MB buffer
        // Record 1000 events — should handle without panic or crash
        for i in 0..1000 {
            bb.record(BlackBoxEvent::DeadlineMiss {
                name: format!("node_{}", i),
                deadline_us: 1000,
                actual_us: 2000,
            });
        }
        let events = bb.events();
        assert!(!events.is_empty(), "should have events");
        assert!(events.len() <= 1000, "should not exceed what was recorded");
    }

    #[test]
    fn test_blackbox_empty_events() {
        let bb = BlackBox::new(64);
        assert!(bb.events().is_empty(), "new blackbox should have no events");
        assert_eq!(bb.get_loss_count(), 0);
        assert!(bb.anomalies().is_empty(), "no anomalies on empty blackbox");
    }

    #[test]
    fn test_blackbox_zero_max_size() {
        let mut bb = BlackBox::new(0);
        // Recording to zero-size blackbox should not panic
        bb.record(BlackBoxEvent::DeadlineMiss {
            name: "test".to_string(),
            deadline_us: 100,
            actual_us: 200,
        });
        // Event either stored (if 0 means unlimited) or lost (if 0 means disabled)
        // Important: no panic
    }

    #[test]
    fn test_blackbox_all_event_types() {
        let mut bb = BlackBox::new(64);

        bb.record(BlackBoxEvent::DeadlineMiss {
            name: "node_a".to_string(),
            deadline_us: 1000,
            actual_us: 2000,
        });
        bb.record(BlackBoxEvent::BudgetViolation {
            name: "node_b".to_string(),
            budget_us: 500,
            actual_us: 800,
        });
        bb.record(BlackBoxEvent::NodeError {
            name: "node_c".to_string(),
            error: "test panic".to_string(),
            severity: crate::error::Severity::Fatal,
        });

        let events = bb.events();
        assert_eq!(events.len(), 3, "all event types should be recorded");

        let anomalies = bb.anomalies();
        assert!(anomalies.len() >= 2, "panics and misses should be anomalies");
    }

    #[test]
    fn test_blackbox_concurrent_record_no_panic() {
        use std::sync::{Arc, Mutex};

        let bb = Arc::new(Mutex::new(BlackBox::new(256)));
        let mut handles = vec![];

        // 4 threads recording simultaneously
        for thread_id in 0..4 {
            let bb_clone = bb.clone();
            handles.push(std::thread::spawn(move || {
                for i in 0..50 {
                    let mut bb = bb_clone.lock().unwrap();
                    bb.record(BlackBoxEvent::DeadlineMiss {
                        name: format!("thread_{}_node_{}", thread_id, i),
                        deadline_us: 1000,
                        actual_us: 2000,
                    });
                }
            }));
        }

        for h in handles {
            h.join().unwrap();
        }

        let bb = bb.lock().unwrap();
        let events = bb.events();
        // Should have some events (up to 256 max)
        assert!(!events.is_empty(), "should have events from concurrent writes");
    }

    // ========================================================================
    // Edge case tests
    // ========================================================================

    /// After wraparound, the buffer should contain exactly the newest entries
    /// and the oldest entries should be gone.
    #[test]
    fn test_wraparound_retains_newest_entries() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 5;

        for i in 0..12u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "wrap".to_string(),
                message: format!("evt_{}", i),
            });
        }

        assert_eq!(bb.len(), 5);
        let events = bb.events();
        // The surviving entries should be the last 5: evt_7 through evt_11
        for (idx, record) in events.iter().enumerate() {
            if let BlackBoxEvent::Custom { message, .. } = &record.event {
                let expected = format!("evt_{}", idx + 7);
                assert_eq!(message, &expected, "entry {} should be {}", idx, expected);
            } else {
                panic!("expected Custom event at index {}", idx);
            }
        }
    }

    /// A BlackBox with capacity 1 should always hold exactly the latest record.
    #[test]
    fn test_capacity_one() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 1;

        bb.record(BlackBoxEvent::Custom {
            category: "cap1".to_string(),
            message: "first".to_string(),
        });
        assert_eq!(bb.len(), 1);
        assert_eq!(bb.get_loss_count(), 0);

        bb.record(BlackBoxEvent::Custom {
            category: "cap1".to_string(),
            message: "second".to_string(),
        });
        assert_eq!(bb.len(), 1);
        assert_eq!(bb.get_loss_count(), 1);

        // The surviving entry must be "second"
        let events = bb.events();
        if let BlackBoxEvent::Custom { message, .. } = &events[0].event {
            assert_eq!(message, "second");
        } else {
            panic!("expected Custom event");
        }

        bb.record(BlackBoxEvent::Custom {
            category: "cap1".to_string(),
            message: "third".to_string(),
        });
        assert_eq!(bb.len(), 1);
        assert_eq!(bb.get_loss_count(), 2);

        let events = bb.events();
        if let BlackBoxEvent::Custom { message, .. } = &events[0].event {
            assert_eq!(message, "third");
        } else {
            panic!("expected Custom event");
        }
    }

    /// Very large node names and messages should be recorded without panic.
    #[test]
    fn test_very_large_entries() {
        let mut bb = BlackBox::new(1);

        let long_name = "n".repeat(10_000);
        let long_msg = "x".repeat(100_000);

        bb.record(BlackBoxEvent::NodeError {
            name: long_name.clone(),
            error: long_msg.clone(),
            severity: crate::error::Severity::Transient,
        });

        assert_eq!(bb.len(), 1);
        let events = bb.events();
        if let BlackBoxEvent::NodeError { name, error, .. } = &events[0].event {
            assert_eq!(name.len(), 10_000);
            assert_eq!(error.len(), 100_000);
        } else {
            panic!("expected NodeError event");
        }
    }

    /// Recording many distinct node names should work; anomalies() should
    /// filter correctly even with heterogeneous names.
    #[test]
    fn test_many_distinct_node_names() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 500;

        for i in 0..200u32 {
            let name = format!("unique_node_{:05}", i);
            // Alternate between a normal tick and an anomaly
            if i % 3 == 0 {
                bb.record(BlackBoxEvent::DeadlineMiss {
                    name,
                    deadline_us: 1000,
                    actual_us: 2000,
                });
            } else {
                bb.record(BlackBoxEvent::NodeTick {
                    name,
                    duration_us: 100,
                    success: true,
                });
            }
        }

        assert_eq!(bb.len(), 200);
        let anomalies = bb.anomalies();
        // Every 3rd entry (i % 3 == 0) is a DeadlineMiss → 0,3,6,...,198 → 67 anomalies
        let expected_anomalies = (0..200u32).filter(|i| i % 3 == 0).count();
        assert_eq!(anomalies.len(), expected_anomalies);
    }

    /// Timestamps should be monotonically non-decreasing across sequential records.
    #[test]
    fn test_timestamp_monotonic_ordering() {
        let mut bb = BlackBox::new(1);

        for i in 0..50u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "ts".to_string(),
                message: i.to_string(),
            });
        }

        let events = bb.events();
        for window in events.windows(2) {
            assert!(
                window[1].timestamp_us >= window[0].timestamp_us,
                "timestamps must be monotonically non-decreasing: {} < {}",
                window[1].timestamp_us,
                window[0].timestamp_us,
            );
        }
    }

    /// tick() increments the tick counter; records should reflect the tick
    /// at which they were recorded.
    #[test]
    fn test_tick_counter_in_records() {
        let mut bb = BlackBox::new(1);

        // Record at tick 0
        bb.record(BlackBoxEvent::Custom {
            category: "tick".to_string(),
            message: "at_tick_0".to_string(),
        });

        bb.tick();
        bb.tick();
        bb.tick(); // now tick_counter = 3

        // Record at tick 3
        bb.record(BlackBoxEvent::Custom {
            category: "tick".to_string(),
            message: "at_tick_3".to_string(),
        });

        let events = bb.events();
        assert_eq!(events.len(), 2);
        assert_eq!(events[0].tick, 0, "first record should be at tick 0");
        assert_eq!(events[1].tick, 3, "second record should be at tick 3");
    }

    /// Loss counter should be accurate after multiple rounds of wraparound.
    #[test]
    fn test_loss_counter_after_multiple_wraparounds() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 4;

        // 4 records: no loss
        for i in 0..4u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "loss".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.len(), 4);
        assert_eq!(bb.get_loss_count(), 0);

        // 4 more: 4 evictions (total records written: 8, capacity: 4)
        for i in 4..8u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "loss".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.len(), 4);
        assert_eq!(bb.get_loss_count(), 4);

        // 100 more: 100 more evictions (total lost: 104)
        for i in 8..108u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "loss".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.len(), 4);
        assert_eq!(bb.get_loss_count(), 104);

        // Buffer should contain the last 4: 104, 105, 106, 107
        let events = bb.events();
        for (idx, record) in events.iter().enumerate() {
            if let BlackBoxEvent::Custom { message, .. } = &record.event {
                let expected = (104 + idx).to_string();
                assert_eq!(message, &expected);
            } else {
                panic!("expected Custom event");
            }
        }
    }

    /// A disabled blackbox (max_size_mb=0) should not record anything and
    /// all queries should return empty.
    #[test]
    fn test_disabled_blackbox_ignores_all_records() {
        let mut bb = BlackBox::new(0);

        bb.record(BlackBoxEvent::EmergencyStop {
            reason: "fire".to_string(),
        });
        bb.record(BlackBoxEvent::NodeError {
            name: "node".to_string(),
            error: "boom".to_string(),
            severity: crate::error::Severity::Fatal,
        });

        assert!(bb.is_empty());
        assert_eq!(bb.len(), 0);
        assert!(bb.events().is_empty());
        assert!(bb.anomalies().is_empty());
        assert_eq!(bb.get_loss_count(), 0);
    }

    /// anomalies() should return BudgetViolation and EmergencyStop events
    /// but not SchedulerStart, SchedulerStop, NodeAdded, NodeTick,
    /// LearningComplete, or Custom.
    #[test]
    fn test_anomalies_filters_all_non_anomaly_types() {
        let mut bb = BlackBox::new(1);

        // Non-anomaly events
        bb.record(BlackBoxEvent::SchedulerStart {
            name: "s".to_string(),
            node_count: 1,
            config: "c".to_string(),
        });
        bb.record(BlackBoxEvent::SchedulerStop {
            reason: "done".to_string(),
            total_ticks: 100,
        });
        bb.record(BlackBoxEvent::NodeAdded {
            name: "n".to_string(),
            order: 0,
        });
        bb.record(BlackBoxEvent::NodeTick {
            name: "n".to_string(),
            duration_us: 50,
            success: true,
        });
        bb.record(BlackBoxEvent::LearningComplete {
            duration_ms: 500,
            tier_summary: "done".to_string(),
        });
        bb.record(BlackBoxEvent::Custom {
            category: "info".to_string(),
            message: "hello".to_string(),
        });

        assert_eq!(bb.anomalies().len(), 0, "none of these should be anomalies");

        // Anomaly events
        bb.record(BlackBoxEvent::NodeError {
            name: "n".to_string(),
            error: "err".to_string(),
            severity: crate::error::Severity::Transient,
        });
        bb.record(BlackBoxEvent::DeadlineMiss {
            name: "n".to_string(),
            deadline_us: 100,
            actual_us: 200,
        });
        bb.record(BlackBoxEvent::BudgetViolation {
            name: "n".to_string(),
            budget_us: 100,
            actual_us: 300,
        });
        bb.record(BlackBoxEvent::EmergencyStop {
            reason: "critical".to_string(),
        });

        assert_eq!(
            bb.anomalies().len(),
            4,
            "all four anomaly types must be returned"
        );
    }

    /// clear() should reset the buffer, tick counter, and loss counter.
    /// Subsequent records should start fresh.
    #[test]
    fn test_clear_resets_all_state() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 3;

        for i in 0..5u32 {
            bb.tick();
            bb.record(BlackBoxEvent::Custom {
                category: "c".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.len(), 3);
        assert_eq!(bb.get_loss_count(), 2);

        bb.clear();

        assert!(bb.is_empty());
        assert_eq!(bb.len(), 0);
        assert_eq!(bb.get_loss_count(), 0);
        assert!(bb.events().is_empty());
        assert!(bb.anomalies().is_empty());

        // Record after clear — tick counter should be 0 again
        bb.record(BlackBoxEvent::Custom {
            category: "after".to_string(),
            message: "cleared".to_string(),
        });
        let events = bb.events();
        assert_eq!(events.len(), 1);
        assert_eq!(events[0].tick, 0, "tick counter must reset to 0 after clear");
    }

    /// save() and load() round-trip: buffer contents and loss counter must
    /// survive serialization.
    #[test]
    fn test_save_load_roundtrip_preserves_data() {
        let dir = std::env::temp_dir().join(format!(
            "horus_bb_roundtrip_test_{}",
            std::process::id()
        ));

        let mut bb = BlackBox::new(1).with_path(dir.clone());
        bb.max_size = 5;

        for i in 0..8u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "rt".to_string(),
                message: format!("item_{}", i),
            });
        }
        assert_eq!(bb.len(), 5);
        assert_eq!(bb.get_loss_count(), 3);

        bb.save().expect("save must succeed");

        // Load into a fresh blackbox
        let mut bb2 = BlackBox::new(1).with_path(dir.clone());
        bb2.load().expect("load must succeed");

        assert_eq!(bb2.len(), 5, "loaded buffer must have same length");
        assert_eq!(
            bb2.get_loss_count(),
            3,
            "loaded loss counter must match saved value"
        );

        // Verify event content survived
        let events = bb2.events();
        for (idx, record) in events.iter().enumerate() {
            if let BlackBoxEvent::Custom { message, .. } = &record.event {
                let expected = format!("item_{}", idx + 3);
                assert_eq!(message, &expected, "event {} content mismatch", idx);
            } else {
                panic!("expected Custom event at index {}", idx);
            }
        }

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// load() on a nonexistent file should be a no-op (no error, no data).
    #[test]
    fn test_load_nonexistent_file_is_noop() {
        let dir = std::env::temp_dir().join(format!(
            "horus_bb_nofile_test_{}",
            std::process::id()
        ));
        // Ensure the dir does NOT exist
        let _ = std::fs::remove_dir_all(&dir);

        let mut bb = BlackBox::new(1).with_path(dir.clone());
        // load() should succeed silently — file doesn't exist
        bb.load().expect("load on missing file must not error");
        assert!(bb.is_empty());
        assert_eq!(bb.get_loss_count(), 0);

        let _ = std::fs::remove_dir_all(&dir);
    }

    /// save() without a persist_path should be a no-op (no error).
    #[test]
    fn test_save_without_path_is_noop() {
        let mut bb = BlackBox::new(1);
        bb.record(BlackBoxEvent::Custom {
            category: "orphan".to_string(),
            message: "no_path".to_string(),
        });
        bb.save().expect("save without path must not error");
    }

    /// The default BlackBox should be disabled and empty.
    #[test]
    fn test_default_is_disabled() {
        let mut bb = BlackBox::default();
        bb.record(BlackBoxEvent::Custom {
            category: "test".to_string(),
            message: "should_be_ignored".to_string(),
        });
        assert!(bb.is_empty(), "default blackbox is disabled; records ignored");
        assert_eq!(bb.get_loss_count(), 0);
    }

    /// with_wal_flush_interval(0) should clamp to 1 (flush every record).
    #[test]
    fn test_wal_flush_interval_clamps_to_one() {
        let bb = BlackBox::new(1).with_wal_flush_interval(0);
        assert_eq!(
            bb.wal_flush_interval, 1,
            "interval 0 should be clamped to 1"
        );
    }

    /// Exact boundary: recording exactly max_size entries should fill the
    /// buffer with zero loss. One more should evict exactly one.
    #[test]
    fn test_exact_capacity_boundary() {
        let mut bb = BlackBox::new(1);
        bb.max_size = 7;

        // Fill exactly to capacity
        for i in 0..7u32 {
            bb.record(BlackBoxEvent::Custom {
                category: "boundary".to_string(),
                message: i.to_string(),
            });
        }
        assert_eq!(bb.len(), 7);
        assert_eq!(bb.get_loss_count(), 0, "no loss when exactly at capacity");

        // One more record should evict the oldest
        bb.record(BlackBoxEvent::Custom {
            category: "boundary".to_string(),
            message: "7".to_string(),
        });
        assert_eq!(bb.len(), 7);
        assert_eq!(bb.get_loss_count(), 1, "exactly one eviction");

        // Oldest surviving entry should be "1" (not "0")
        let events = bb.events();
        if let BlackBoxEvent::Custom { message, .. } = &events[0].event {
            assert_eq!(message, "1", "entry 0 was evicted; first surviving is 1");
        } else {
            panic!("expected Custom event");
        }
    }

    /// Interleaving tick() and record() — tick counter in records should
    /// match the counter at the time of recording, not at retrieval time.
    #[test]
    fn test_interleaved_tick_and_record() {
        let mut bb = BlackBox::new(1);

        // tick, record, tick, tick, record, record
        bb.tick(); // tick_counter = 1
        bb.record(BlackBoxEvent::Custom {
            category: "il".to_string(),
            message: "a".to_string(),
        });
        bb.tick(); // tick_counter = 2
        bb.tick(); // tick_counter = 3
        bb.record(BlackBoxEvent::Custom {
            category: "il".to_string(),
            message: "b".to_string(),
        });
        bb.record(BlackBoxEvent::Custom {
            category: "il".to_string(),
            message: "c".to_string(),
        });

        let events = bb.events();
        assert_eq!(events.len(), 3);
        assert_eq!(events[0].tick, 1);
        assert_eq!(events[1].tick, 3);
        assert_eq!(events[2].tick, 3, "two records at same tick");
    }

    /// Anomalies should preserve ordering even when mixed with non-anomaly events.
    #[test]
    fn test_anomalies_preserve_insertion_order() {
        let mut bb = BlackBox::new(1);

        bb.record(BlackBoxEvent::NodeTick {
            name: "ok".to_string(),
            duration_us: 10,
            success: true,
        });
        bb.tick();

        bb.record(BlackBoxEvent::DeadlineMiss {
            name: "first_miss".to_string(),
            deadline_us: 100,
            actual_us: 200,
        });
        bb.tick();

        bb.record(BlackBoxEvent::Custom {
            category: "info".to_string(),
            message: "filler".to_string(),
        });
        bb.tick();

        bb.record(BlackBoxEvent::EmergencyStop {
            reason: "overheat".to_string(),
        });
        bb.tick();

        bb.record(BlackBoxEvent::BudgetViolation {
            name: "budget_node".to_string(),
            budget_us: 50,
            actual_us: 150,
        });

        let anomalies = bb.anomalies();
        assert_eq!(anomalies.len(), 3);
        // Verify tick ordering matches insertion order
        assert_eq!(anomalies[0].tick, 1, "DeadlineMiss at tick 1");
        assert_eq!(anomalies[1].tick, 3, "EmergencyStop at tick 3");
        assert_eq!(anomalies[2].tick, 4, "BudgetViolation at tick 4");
    }
}
