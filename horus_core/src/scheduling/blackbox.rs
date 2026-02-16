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
use std::sync::{Arc, Mutex};
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
    NodeAdded { name: String, priority: u32 },
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
        }
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

        // Write to WAL first (for crash recovery)
        if let Some(ref mut wal) = self.wal_file {
            if let Ok(json) = serde_json::to_string(&record) {
                let _ = writeln!(wal, "{}", json);
                let _ = wal.flush();
            }
        }

        // Add to circular buffer
        if self.buffer.len() >= self.max_size {
            self.buffer.pop_front();
        }
        self.buffer.push_back(record);
    }

    /// Increment tick counter (call once per scheduler tick)
    pub fn tick(&mut self) {
        self.tick_counter += 1;
    }

    /// Get all recorded events
    pub fn get_events(&self) -> Vec<BlackBoxRecord> {
        self.buffer.iter().cloned().collect()
    }

    /// Get all errors and warnings
    pub fn get_anomalies(&self) -> Vec<BlackBoxRecord> {
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

    /// Save buffer to disk
    pub fn save(&self) -> std::io::Result<()> {
        if let Some(ref path) = self.persist_path {
            let file = File::create(path)?;
            let writer = BufWriter::new(file);
            serde_json::to_writer_pretty(writer, &self.get_events())?;
            info!(
                "[BLACKBOX] Saved {} events to {:?}",
                self.buffer.len(),
                path
            );
        }
        Ok(())
    }

    /// Load buffer from disk
    pub fn load(&mut self) -> std::io::Result<()> {
        if let Some(ref path) = self.persist_path {
            if path.exists() {
                let content = fs::read_to_string(path)?;
                let events: Vec<BlackBoxRecord> = serde_json::from_str(&content)?;
                self.buffer = VecDeque::from(events);
                info!(
                    "[BLACKBOX] Loaded {} events from {:?}",
                    self.buffer.len(),
                    path
                );
            }
        }
        Ok(())
    }

    /// Clear the buffer
    pub fn clear(&mut self) {
        self.buffer.clear();
        self.tick_counter = 0;
    }

    /// Get buffer size
    pub fn len(&self) -> usize {
        self.buffer.len()
    }

    /// Check if buffer is empty
    pub fn is_empty(&self) -> bool {
        self.buffer.is_empty()
    }
}

impl Default for BlackBox {
    fn default() -> Self {
        Self::new(0) // Disabled by default
    }
}

/// Thread-safe black box wrapper
pub type SharedBlackBox = Arc<Mutex<BlackBox>>;

/// Create a thread-safe black box
pub fn create_shared_blackbox(max_size_mb: usize) -> SharedBlackBox {
    Arc::new(Mutex::new(BlackBox::new(max_size_mb)))
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

        let anomalies = bb.get_anomalies();
        assert_eq!(anomalies.len(), 2);
    }
}
