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
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::sync::Arc;
use std::time::{Duration, Instant};

use parking_lot::Mutex;
use serde::{Deserialize, Serialize};
use thiserror::Error;

use crate::{Node, NodeInfo};

/// Errors in deterministic execution
#[derive(Error, Debug)]
pub enum DeterministicError {
    #[error("Determinism violation at tick {tick}: {message}")]
    DeterminismViolation { tick: u64, message: String },

    #[error("Execution diverged at tick {tick}: expected {expected:?}, got {actual:?}")]
    ExecutionDiverged {
        tick: u64,
        expected: Vec<u8>,
        actual: Vec<u8>,
    },

    #[error("Non-deterministic operation detected: {0}")]
    NonDeterministicOp(String),

    #[error("Verification failed: {0}")]
    VerificationFailed(String),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),
}

pub type Result<T> = std::result::Result<T, DeterministicError>;

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

    /// Create config for replay (no tracing overhead)
    pub fn for_replay(seed: u64, tick_duration_ns: u64) -> Self {
        Self {
            seed,
            virtual_time: true,
            tick_duration_ns,
            record_trace: false,
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

    /// Set specific tick (for replay)
    pub fn set_tick(&self, tick: u64) {
        self.tick.store(tick, Ordering::Release);
        if self.use_virtual_time {
            self.virtual_time_ns
                .store(tick * self.tick_duration_ns, Ordering::Release);
        }
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

    /// Get deterministic random f64 in [0, 1)
    pub fn random_f64(&self) -> f64 {
        (self.random_u64() as f64) / (u64::MAX as f64)
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

/// Registered node in deterministic scheduler
struct DeterministicNode {
    node: Box<dyn Node>,
    name: String,
    priority: u32,
    enabled: bool,
    node_info: NodeInfo,
}

/// Deterministic scheduler that guarantees reproducible execution
pub struct DeterministicScheduler {
    /// Configuration
    config: DeterministicConfig,
    /// Deterministic clock
    clock: Arc<DeterministicClock>,
    /// Registered nodes
    nodes: Vec<DeterministicNode>,
    /// Execution trace (if recording)
    trace: Option<Mutex<ExecutionTrace>>,
    /// Running flag
    running: AtomicBool,
    /// Maximum ticks (0 = unlimited)
    max_ticks: u64,
}

impl DeterministicScheduler {
    /// Create a new deterministic scheduler
    pub fn new(config: DeterministicConfig) -> Self {
        let clock = Arc::new(DeterministicClock::new(&config));
        let trace = if config.record_trace {
            Some(Mutex::new(ExecutionTrace::new(config.clone())))
        } else {
            None
        };

        Self {
            config,
            clock,
            nodes: Vec::new(),
            trace,
            running: AtomicBool::new(false),
            max_ticks: 0,
        }
    }

    /// Create with default config
    pub fn with_seed(seed: u64) -> Self {
        let config = DeterministicConfig {
            seed,
            ..Default::default()
        };
        Self::new(config)
    }

    /// Add a node to the scheduler
    pub fn add(&mut self, node: Box<dyn Node>, priority: u32) {
        let name = node.name().to_string();
        let node_info = NodeInfo::new(name.clone(), false);
        self.nodes.push(DeterministicNode {
            node,
            name,
            priority,
            enabled: true,
            node_info,
        });
        // Sort by priority
        self.nodes.sort_by_key(|n| n.priority);
    }

    /// Set maximum ticks to run
    pub fn set_max_ticks(&mut self, max: u64) {
        self.max_ticks = max;
    }

    /// Get the deterministic clock
    pub fn clock(&self) -> Arc<DeterministicClock> {
        self.clock.clone()
    }

    /// Run the scheduler
    pub fn run(&mut self) -> Result<()> {
        self.running.store(true, Ordering::Release);

        // Initialize all nodes
        for (idx, dn) in self.nodes.iter_mut().enumerate() {
            let start = std::time::Instant::now();
            let _ = dn.node.init(&mut dn.node_info);
            let duration = start.elapsed();

            if let Some(ref trace) = self.trace {
                trace.lock().add(TraceEntry {
                    tick: 0,
                    node_index: idx,
                    node_name: dn.name.clone(),
                    entry_type: TraceEntryType::TickStart,
                    timestamp_ns: self.clock.now_ns(),
                    duration_ns: duration.as_nanos() as u64,
                    input_hash: None,
                    output_hash: None,
                    data: None,
                });
            }
        }

        // Main loop
        while self.running.load(Ordering::Acquire) {
            let tick = self.clock.tick();

            // Check max ticks
            if self.max_ticks > 0 && tick >= self.max_ticks {
                break;
            }

            // Execute all nodes in priority order
            for (idx, dn) in self.nodes.iter_mut().enumerate() {
                if !dn.enabled {
                    continue;
                }

                let start = std::time::Instant::now();

                // Record tick start
                if let Some(ref trace) = self.trace {
                    trace.lock().add(TraceEntry {
                        tick,
                        node_index: idx,
                        node_name: dn.name.clone(),
                        entry_type: TraceEntryType::TickStart,
                        timestamp_ns: self.clock.now_ns(),
                        duration_ns: 0,
                        input_hash: None,
                        output_hash: None,
                        data: None,
                    });
                }

                // Execute node
                dn.node.tick();

                let duration = start.elapsed();

                // Record tick end
                if let Some(ref trace) = self.trace {
                    trace.lock().add(TraceEntry {
                        tick,
                        node_index: idx,
                        node_name: dn.name.clone(),
                        entry_type: TraceEntryType::TickEnd,
                        timestamp_ns: self.clock.now_ns(),
                        duration_ns: duration.as_nanos() as u64,
                        input_hash: None,
                        output_hash: None,
                        data: None,
                    });
                }
            }

            // Finalize tick in trace
            if let Some(ref trace) = self.trace {
                trace.lock().finalize_tick(tick);
            }

            // Advance clock
            self.clock.advance_tick();
        }

        // Shutdown nodes
        for dn in self.nodes.iter_mut() {
            let _ = dn.node.shutdown(&mut dn.node_info);
        }

        Ok(())
    }

    /// Run for a specific number of ticks
    pub fn run_ticks(&mut self, ticks: u64) -> Result<()> {
        self.max_ticks = self.clock.tick() + ticks;
        self.run()
    }

    /// Stop the scheduler
    pub fn stop(&self) {
        self.running.store(false, Ordering::Release);
    }

    /// Get the execution trace
    pub fn trace(&self) -> Option<ExecutionTrace> {
        self.trace.as_ref().map(|t| t.lock().clone())
    }

    /// Reset scheduler state for re-run
    pub fn reset(&mut self) {
        self.clock.reset();
        if let Some(ref trace) = self.trace {
            *trace.lock() = ExecutionTrace::new(self.config.clone());
        }
    }
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
    use crate::error::HorusResult;

    struct CounterNode {
        count: u64,
    }

    impl Node for CounterNode {
        fn name(&self) -> &'static str {
            "counter"
        }

        fn init(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
            self.count = 0;
            Ok(())
        }

        fn tick(&mut self) {
            self.count += 1;
        }

        fn shutdown(&mut self, _ctx: &mut NodeInfo) -> HorusResult<()> {
            Ok(())
        }
    }

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
    fn test_deterministic_scheduler() {
        let config = DeterministicConfig::with_trace(42);
        let mut scheduler = DeterministicScheduler::new(config);

        scheduler.add(Box::new(CounterNode { count: 0 }), 10);
        scheduler.set_max_ticks(10);
        scheduler.run().unwrap();

        let trace = scheduler.trace().unwrap();
        assert_eq!(trace.total_ticks, 10);
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

    #[test]
    fn test_timing_bounds() {
        let mut bounds = TimingBounds::new(1000); // 1ms WCET

        // Add normal executions
        for _ in 0..10 {
            bounds.record_execution(500_000); // 500us
        }

        assert!(bounds.check_compliance().is_ok());

        // Add violation
        bounds.record_execution(2_000_000); // 2ms - exceeds WCET

        assert!(bounds.check_compliance().is_err());
    }

    #[test]
    fn test_timing_bounds_per_node() {
        let mut bounds = TimingBounds::new(1000); // 1ms WCET

        // Record executions for different nodes
        bounds.record_node_execution("sensor", 0, 200_000);
        bounds.record_node_execution("sensor", 1, 300_000);
        bounds.record_node_execution("control", 0, 800_000);
        bounds.record_node_execution("control", 1, 1_500_000); // violation

        assert_eq!(bounds.violations.len(), 1);
        assert_eq!(bounds.violations[0].node_name, "control");
        assert_eq!(bounds.violations[0].tick, 1);
    }
}

// ============================================================================
// WCET Timing Analysis
// ============================================================================

/// Timing bounds for WCET verification
#[derive(Debug, Clone)]
pub struct TimingBounds {
    /// Specified WCET in microseconds
    pub wcet_us: u64,
    /// Observed maximum execution time
    pub observed_max_ns: u64,
    /// Observed minimum execution time
    pub observed_min_ns: u64,
    /// Sum of all execution times (for average)
    pub total_ns: u64,
    /// Number of executions recorded
    pub count: u64,
    /// All recorded violations
    pub violations: Vec<TimingViolation>,
    /// Per-node timing data
    pub node_timings: std::collections::HashMap<String, NodeTimingStats>,
}

/// Statistics for a single node's timing
#[derive(Debug, Clone, Default, Serialize, Deserialize)]
pub struct NodeTimingStats {
    pub min_ns: u64,
    pub max_ns: u64,
    pub total_ns: u64,
    pub count: u64,
    pub violations: u32,
}

impl NodeTimingStats {
    pub fn avg_ns(&self) -> u64 {
        if self.count > 0 {
            self.total_ns / self.count
        } else {
            0
        }
    }
}

/// A timing violation event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingViolation {
    pub tick: u64,
    pub node_name: String,
    pub expected_us: u64,
    pub actual_ns: u64,
    pub severity: ViolationSeverity,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum ViolationSeverity {
    /// < 10% over WCET
    Minor,
    /// 10-50% over WCET
    Major,
    /// > 50% over WCET
    Critical,
}

impl TimingBounds {
    pub fn new(wcet_us: u64) -> Self {
        Self {
            wcet_us,
            observed_max_ns: 0,
            observed_min_ns: u64::MAX,
            total_ns: 0,
            count: 0,
            violations: Vec::new(),
            node_timings: std::collections::HashMap::new(),
        }
    }

    /// Record an execution time
    pub fn record_execution(&mut self, duration_ns: u64) {
        self.observed_max_ns = self.observed_max_ns.max(duration_ns);
        self.observed_min_ns = self.observed_min_ns.min(duration_ns);
        self.total_ns += duration_ns;
        self.count += 1;
    }

    /// Record execution for a specific node
    pub fn record_node_execution(&mut self, node_name: &str, tick: u64, duration_ns: u64) {
        let wcet_ns = self.wcet_us * 1000;

        let stats = self.node_timings.entry(node_name.to_string()).or_default();
        if stats.count == 0 {
            stats.min_ns = duration_ns;
            stats.max_ns = duration_ns;
        } else {
            stats.min_ns = stats.min_ns.min(duration_ns);
            stats.max_ns = stats.max_ns.max(duration_ns);
        }
        stats.total_ns += duration_ns;
        stats.count += 1;

        if duration_ns > wcet_ns {
            stats.violations += 1;
            let overrun_percent = ((duration_ns as f64 / wcet_ns as f64) - 1.0) * 100.0;
            let severity = if overrun_percent < 10.0 {
                ViolationSeverity::Minor
            } else if overrun_percent < 50.0 {
                ViolationSeverity::Major
            } else {
                ViolationSeverity::Critical
            };

            self.violations.push(TimingViolation {
                tick,
                node_name: node_name.to_string(),
                expected_us: self.wcet_us,
                actual_ns: duration_ns,
                severity,
            });
        }

        self.record_execution(duration_ns);
    }

    /// Check if timing is compliant with WCET
    pub fn check_compliance(&self) -> Result<()> {
        let wcet_ns = self.wcet_us * 1000;
        if self.observed_max_ns > wcet_ns {
            return Err(DeterministicError::VerificationFailed(format!(
                "WCET violation: observed {}ns > {}ns limit",
                self.observed_max_ns, wcet_ns
            )));
        }
        Ok(())
    }

    /// Get average execution time in nanoseconds
    pub fn average_ns(&self) -> u64 {
        if self.count > 0 {
            self.total_ns / self.count
        } else {
            0
        }
    }

    /// Compute statistical metrics
    pub fn compute_statistics(&self) -> TimingStatistics {
        TimingStatistics {
            wcet_us: self.wcet_us,
            observed_max_ns: self.observed_max_ns,
            observed_min_ns: if self.observed_min_ns == u64::MAX {
                0
            } else {
                self.observed_min_ns
            },
            average_ns: self.average_ns(),
            sample_count: self.count,
            violation_count: self.violations.len(),
            wcet_margin_percent: if self.observed_max_ns > 0 {
                ((self.wcet_us as f64 * 1000.0) / self.observed_max_ns as f64 - 1.0) * 100.0
            } else {
                100.0
            },
        }
    }
}

/// Statistical summary of timing data
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimingStatistics {
    pub wcet_us: u64,
    pub observed_max_ns: u64,
    pub observed_min_ns: u64,
    pub average_ns: u64,
    pub sample_count: u64,
    pub violation_count: usize,
    /// How much margin remains before hitting WCET (negative = over WCET)
    pub wcet_margin_percent: f64,
}
