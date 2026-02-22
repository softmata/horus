//! Deterministic Execution System
//!
//! Provides reproducible execution of node graphs using virtual time
//! and seeded RNG. Useful for debugging and regression testing.
//!
//! ## Key Features
//!
//! - **Deterministic Clock**: Virtual time that advances predictably
//! - **Seeded RNG**: All randomness derived from seed
//! - **Execution Trace**: Record of node executions with timing
//! - **Trace Comparison**: Compare two runs to find divergence

use std::hash::{Hash, Hasher};
use std::sync::atomic::{AtomicU64, Ordering};
use std::time::{Duration, Instant};

use serde::{Deserialize, Serialize};

/// Configuration for deterministic execution
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DeterministicConfig {
    /// Seed for deterministic RNG
    pub seed: u64,
    /// Whether to use virtual time (ignore wall clock)
    pub virtual_time: bool,
    /// Fixed tick duration in nanoseconds (for virtual time)
    pub tick_duration_ns: u64,
    /// Whether to record execution trace
    pub record_trace: bool,
}

impl Default for DeterministicConfig {
    fn default() -> Self {
        Self {
            seed: 42,
            virtual_time: true,
            tick_duration_ns: 1_000_000, // 1ms per tick
            record_trace: true,
        }
    }
}

impl DeterministicConfig {
    /// Create config with tracing enabled
    pub fn with_trace(seed: u64) -> Self {
        Self {
            seed,
            virtual_time: true,
            tick_duration_ns: 1_000_000,
            record_trace: true,
        }
    }
}

/// Deterministic clock that provides reproducible time
#[derive(Debug)]
pub struct DeterministicClock {
    /// Current virtual time in nanoseconds
    virtual_time_ns: AtomicU64,
    /// Tick duration in nanoseconds
    tick_duration_ns: u64,
    /// Current tick number
    tick: AtomicU64,
    /// Whether to use virtual time
    use_virtual_time: bool,
    /// Real start time (for hybrid mode)
    real_start: Instant,
    /// Seed for RNG
    seed: u64,
    /// Current RNG state
    rng_state: AtomicU64,
}

impl DeterministicClock {
    /// Create a new deterministic clock
    pub fn new(config: &DeterministicConfig) -> Self {
        Self {
            virtual_time_ns: AtomicU64::new(0),
            tick_duration_ns: config.tick_duration_ns,
            tick: AtomicU64::new(0),
            use_virtual_time: config.virtual_time,
            real_start: Instant::now(),
            seed: config.seed,
            rng_state: AtomicU64::new(config.seed),
        }
    }

    /// Get current time in nanoseconds
    pub fn now_ns(&self) -> u64 {
        if self.use_virtual_time {
            self.virtual_time_ns.load(Ordering::Acquire)
        } else {
            self.real_start.elapsed().as_nanos() as u64
        }
    }

    /// Get current time as Duration
    pub fn now(&self) -> Duration {
        Duration::from_nanos(self.now_ns())
    }

    /// Get current tick number
    pub fn tick(&self) -> u64 {
        self.tick.load(Ordering::Acquire)
    }

    /// Advance to next tick
    pub fn advance_tick(&self) -> u64 {
        let new_tick = self.tick.fetch_add(1, Ordering::AcqRel) + 1;
        if self.use_virtual_time {
            self.virtual_time_ns
                .fetch_add(self.tick_duration_ns, Ordering::AcqRel);
        }
        new_tick
    }

    /// Get deterministic random number
    pub fn random_u64(&self) -> u64 {
        // Simple xorshift64 for deterministic randomness
        let mut state = self.rng_state.load(Ordering::Acquire);
        state ^= state << 13;
        state ^= state >> 7;
        state ^= state << 17;
        self.rng_state.store(state, Ordering::Release);
        state
    }

    /// Reset clock to initial state
    pub fn reset(&self) {
        self.tick.store(0, Ordering::Release);
        self.virtual_time_ns.store(0, Ordering::Release);
        self.rng_state.store(self.seed, Ordering::Release);
    }

    /// Get seed
    pub fn seed(&self) -> u64 {
        self.seed
    }
}

/// Entry in the execution trace
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TraceEntry {
    /// Tick number
    pub tick: u64,
    /// Node index
    pub node_index: usize,
    /// Node name
    pub node_name: String,
    /// Entry type
    pub entry_type: TraceEntryType,
    /// Timestamp (virtual or real)
    pub timestamp_ns: u64,
    /// Duration of operation in nanoseconds
    pub duration_ns: u64,
    /// Hash of inputs (if any)
    pub input_hash: Option<u64>,
    /// Hash of outputs (if any)
    pub output_hash: Option<u64>,
    /// Raw data (optional, for debugging)
    pub data: Option<Vec<u8>>,
}

/// Type of trace entry
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum TraceEntryType {
    /// Node tick started
    TickStart,
    /// Node tick completed
    TickEnd,
    /// Input received
    Input,
    /// Output produced
    Output,
    /// State change
    StateChange,
    /// Error occurred
    Error,
    /// Custom event
    Custom,
}

/// Complete execution trace
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct ExecutionTrace {
    /// Configuration used
    pub config: DeterministicConfig,
    /// All trace entries
    pub entries: Vec<TraceEntry>,
    /// Per-tick hashes for quick comparison
    pub tick_hashes: Vec<u64>,
    /// Total execution time
    pub total_duration_ns: u64,
    /// Number of ticks executed
    pub total_ticks: u64,
}

impl ExecutionTrace {
    pub fn new(config: DeterministicConfig) -> Self {
        Self {
            config,
            entries: Vec::new(),
            tick_hashes: Vec::new(),
            total_duration_ns: 0,
            total_ticks: 0,
        }
    }

    /// Add entry to trace
    pub fn add(&mut self, entry: TraceEntry) {
        self.entries.push(entry);
    }

    /// Finalize tick and compute hash
    pub fn finalize_tick(&mut self, tick: u64) {
        // Compute hash of all entries for this tick
        let mut hasher = std::collections::hash_map::DefaultHasher::new();
        for entry in self.entries.iter().filter(|e| e.tick == tick) {
            entry.tick.hash(&mut hasher);
            entry.node_index.hash(&mut hasher);
            (entry.entry_type as u8).hash(&mut hasher);
            if let Some(h) = entry.output_hash {
                h.hash(&mut hasher);
            }
        }
        self.tick_hashes.push(hasher.finish());
        self.total_ticks = tick + 1;
    }

    /// Compare with another trace for divergence
    pub fn compare(&self, other: &ExecutionTrace) -> Option<DivergenceInfo> {
        // Quick check using tick hashes
        let min_ticks = self.tick_hashes.len().min(other.tick_hashes.len());

        for i in 0..min_ticks {
            if self.tick_hashes[i] != other.tick_hashes[i] {
                // Found divergence, get detailed info
                let tick = i as u64;
                let self_entries: Vec<_> = self.entries.iter().filter(|e| e.tick == tick).collect();
                let other_entries: Vec<_> =
                    other.entries.iter().filter(|e| e.tick == tick).collect();

                return Some(DivergenceInfo {
                    tick,
                    self_hash: self.tick_hashes[i],
                    other_hash: other.tick_hashes[i],
                    self_entry_count: self_entries.len(),
                    other_entry_count: other_entries.len(),
                    message: format!(
                        "Tick {} diverged: {} entries vs {} entries",
                        tick,
                        self_entries.len(),
                        other_entries.len()
                    ),
                });
            }
        }

        // Check if lengths differ
        if self.tick_hashes.len() != other.tick_hashes.len() {
            return Some(DivergenceInfo {
                tick: min_ticks as u64,
                self_hash: 0,
                other_hash: 0,
                self_entry_count: self.tick_hashes.len(),
                other_entry_count: other.tick_hashes.len(),
                message: format!(
                    "Different number of ticks: {} vs {}",
                    self.tick_hashes.len(),
                    other.tick_hashes.len()
                ),
            });
        }

        None
    }

    /// Save trace to file
    pub fn save(&self, path: &std::path::Path) -> std::io::Result<()> {
        let file = std::fs::File::create(path)?;
        serde_json::to_writer_pretty(file, self).map_err(std::io::Error::other)
    }

    /// Load trace from file
    pub fn load(path: &std::path::Path) -> std::io::Result<Self> {
        let file = std::fs::File::open(path)?;
        serde_json::from_reader(file).map_err(std::io::Error::other)
    }
}

/// Information about where execution diverged
#[derive(Debug, Clone)]
pub struct DivergenceInfo {
    pub tick: u64,
    pub self_hash: u64,
    pub other_hash: u64,
    pub self_entry_count: usize,
    pub other_entry_count: usize,
    pub message: String,
}

impl Serialize for DivergenceInfo {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("DivergenceInfo", 6)?;
        state.serialize_field("tick", &self.tick)?;
        state.serialize_field("self_hash", &self.self_hash)?;
        state.serialize_field("other_hash", &self.other_hash)?;
        state.serialize_field("self_entry_count", &self.self_entry_count)?;
        state.serialize_field("other_entry_count", &self.other_entry_count)?;
        state.serialize_field("message", &self.message)?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for DivergenceInfo {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct DivergenceInfoHelper {
            tick: u64,
            self_hash: u64,
            other_hash: u64,
            self_entry_count: usize,
            other_entry_count: usize,
            message: String,
        }

        let helper = DivergenceInfoHelper::deserialize(deserializer)?;
        Ok(DivergenceInfo {
            tick: helper.tick,
            self_hash: helper.self_hash,
            other_hash: helper.other_hash,
            self_entry_count: helper.self_entry_count,
            other_entry_count: helper.other_entry_count,
            message: helper.message,
        })
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_deterministic_clock() {
        let config = DeterministicConfig::default();
        let clock = DeterministicClock::new(&config);

        assert_eq!(clock.tick(), 0);
        assert_eq!(clock.now_ns(), 0);

        clock.advance_tick();
        assert_eq!(clock.tick(), 1);
        assert_eq!(clock.now_ns(), config.tick_duration_ns);

        clock.advance_tick();
        assert_eq!(clock.tick(), 2);
        assert_eq!(clock.now_ns(), config.tick_duration_ns * 2);
    }

    #[test]
    fn test_deterministic_rng() {
        let config = DeterministicConfig::default();
        let clock1 = DeterministicClock::new(&config);
        let clock2 = DeterministicClock::new(&config);

        // Same seed should produce same sequence
        let seq1: Vec<u64> = (0..10).map(|_| clock1.random_u64()).collect();
        let seq2: Vec<u64> = (0..10).map(|_| clock2.random_u64()).collect();

        assert_eq!(seq1, seq2);
    }

    #[test]
    fn test_trace_comparison() {
        let config = DeterministicConfig::with_trace(42);

        let mut trace1 = ExecutionTrace::new(config.clone());
        let mut trace2 = ExecutionTrace::new(config);

        // Add identical entries
        for tick in 0..5 {
            trace1.add(TraceEntry {
                tick,
                node_index: 0,
                node_name: "test".to_string(),
                entry_type: TraceEntryType::TickEnd,
                timestamp_ns: tick * 1000,
                duration_ns: 100,
                input_hash: None,
                output_hash: Some(tick * 42),
                data: None,
            });
            trace1.finalize_tick(tick);

            trace2.add(TraceEntry {
                tick,
                node_index: 0,
                node_name: "test".to_string(),
                entry_type: TraceEntryType::TickEnd,
                timestamp_ns: tick * 1000,
                duration_ns: 100,
                input_hash: None,
                output_hash: Some(tick * 42),
                data: None,
            });
            trace2.finalize_tick(tick);
        }

        // Should not diverge
        assert!(trace1.compare(&trace2).is_none());
    }

    #[test]
    fn test_trace_divergence_detection() {
        let config = DeterministicConfig::with_trace(42);

        let mut trace1 = ExecutionTrace::new(config.clone());
        let mut trace2 = ExecutionTrace::new(config);

        // Add different entries at tick 2
        for tick in 0..5 {
            trace1.add(TraceEntry {
                tick,
                node_index: 0,
                node_name: "test".to_string(),
                entry_type: TraceEntryType::TickEnd,
                timestamp_ns: tick * 1000,
                duration_ns: 100,
                input_hash: None,
                output_hash: Some(tick * 42),
                data: None,
            });
            trace1.finalize_tick(tick);

            let output_hash = if tick == 2 { 999 } else { tick * 42 };
            trace2.add(TraceEntry {
                tick,
                node_index: 0,
                node_name: "test".to_string(),
                entry_type: TraceEntryType::TickEnd,
                timestamp_ns: tick * 1000,
                duration_ns: 100,
                input_hash: None,
                output_hash: Some(output_hash),
                data: None,
            });
            trace2.finalize_tick(tick);
        }

        // Should diverge at tick 2
        let div = trace1.compare(&trace2);
        assert!(div.is_some());
        assert_eq!(div.unwrap().tick, 2);
    }
}
