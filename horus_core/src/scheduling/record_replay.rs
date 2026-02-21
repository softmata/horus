//! Record/Replay System for HORUS
//!
//! Enables node-level granular recording and replay for debugging,
//! testing, and analysis. Features include:
//! - Record individual nodes or entire system
//! - Replay with tick-perfect determinism
//! - Mix recordings from different runs
//! - Time travel to specific ticks

use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::fs::{self, File};
use std::io::{BufReader, BufWriter};
use std::path::PathBuf;
use std::time::SystemTime;

/// Directory for storing recordings
const RECORDINGS_DIR: &str = ".horus/recordings";

/// Recording file extension
const RECORDING_EXT: &str = "horus";

/// Maximum recording size (100MB per node by default)
const MAX_RECORDING_SIZE: usize = 100 * 1024 * 1024;

/// Recording configuration
#[derive(Debug, Clone)]
pub struct RecordingConfig {
    /// Session name
    pub session_name: String,
    /// Base directory for recordings
    pub base_dir: PathBuf,
    /// Maximum recording size per node
    pub max_size: usize,
    /// Whether to compress recordings
    pub compress: bool,
    /// Record interval (record every N ticks, 1 = every tick)
    pub interval: u64,
    /// Nodes to include in recording (empty = all nodes)
    pub include_nodes: Vec<String>,
    /// Nodes to exclude from recording
    pub exclude_nodes: Vec<String>,
}

impl Default for RecordingConfig {
    fn default() -> Self {
        let base_dir = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(RECORDINGS_DIR);

        Self {
            session_name: format!(
                "recording_{}",
                SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs()
            ),
            base_dir,
            max_size: MAX_RECORDING_SIZE,
            compress: true,
            interval: 1,
            include_nodes: vec![],
            exclude_nodes: vec![],
        }
    }
}

impl RecordingConfig {
    /// Create a new recording config with the given session name
    pub fn new(session_name: String) -> Self {
        Self {
            session_name,
            ..Default::default()
        }
    }

    /// Create config with a named session
    pub fn with_name(name: &str) -> Self {
        Self::new(name.to_string())
    }

    /// Check if a node should be recorded based on include/exclude filters
    pub fn should_record_node(&self, node_name: &str) -> bool {
        // If exclude list is not empty and node is in it, don't record
        if !self.exclude_nodes.is_empty() && self.exclude_nodes.contains(&node_name.to_string()) {
            return false;
        }

        // If include list is not empty, only record nodes in it
        if !self.include_nodes.is_empty() {
            return self.include_nodes.contains(&node_name.to_string());
        }

        // Default: record all nodes
        true
    }

    /// Get the session directory
    pub fn session_dir(&self) -> PathBuf {
        self.base_dir.join(&self.session_name)
    }

    /// Get the path for a node recording
    pub fn node_path(&self, node_name: &str, node_id: &str) -> PathBuf {
        self.session_dir()
            .join(format!("{}@{}.{}", node_name, node_id, RECORDING_EXT))
    }

    /// Get the path for the scheduler recording
    pub fn scheduler_path(&self, scheduler_id: &str) -> PathBuf {
        self.session_dir()
            .join(format!("scheduler@{}.{}", scheduler_id, RECORDING_EXT))
    }
}

/// Convert YAML-based recording config to runtime recording config.
///
/// This bridges the declarative `SchedulerConfig` preset system with the
/// runtime `RecordingConfig` used by `NodeRecorder` and `Scheduler::apply_config()`.
impl From<super::config::RecordingConfigYaml> for RecordingConfig {
    fn from(yaml: super::config::RecordingConfigYaml) -> Self {
        let base_dir = yaml
            .output_dir
            .map(PathBuf::from)
            .unwrap_or_else(|| {
                dirs::home_dir()
                    .unwrap_or_else(|| PathBuf::from("."))
                    .join(RECORDINGS_DIR)
            });

        let session_name = yaml.session_name.unwrap_or_else(|| {
            format!(
                "recording_{}",
                SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap_or_default()
                    .as_secs()
            )
        });

        Self {
            session_name,
            base_dir,
            max_size: if yaml.max_size_mb > 0 {
                yaml.max_size_mb * 1024 * 1024
            } else {
                MAX_RECORDING_SIZE
            },
            compress: yaml.compress,
            interval: yaml.interval as u64,
            include_nodes: yaml.include_nodes,
            exclude_nodes: yaml.exclude_nodes,
        }
    }
}

/// A snapshot of a node's state at a specific tick
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeTickSnapshot {
    /// Tick number
    pub tick: u64,
    /// Timestamp (microseconds since epoch)
    pub timestamp_us: u64,
    /// Inputs received this tick (topic -> serialized data)
    pub inputs: HashMap<String, Vec<u8>>,
    /// Outputs produced this tick (topic -> serialized data)
    pub outputs: HashMap<String, Vec<u8>>,
    /// Internal state snapshot (optional)
    pub state: Option<Vec<u8>>,
    /// Execution duration (nanoseconds)
    pub duration_ns: u64,
}

impl NodeTickSnapshot {
    pub fn new(tick: u64) -> Self {
        Self {
            tick,
            timestamp_us: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            inputs: HashMap::new(),
            outputs: HashMap::new(),
            state: None,
            duration_ns: 0,
        }
    }

    pub fn with_input(mut self, topic: &str, data: Vec<u8>) -> Self {
        self.inputs.insert(topic.to_string(), data);
        self
    }

    pub fn with_output(mut self, topic: &str, data: Vec<u8>) -> Self {
        self.outputs.insert(topic.to_string(), data);
        self
    }

    pub fn with_state(mut self, state: Vec<u8>) -> Self {
        self.state = Some(state);
        self
    }

    pub fn with_duration(mut self, duration_ns: u64) -> Self {
        self.duration_ns = duration_ns;
        self
    }
}

/// Recording of a single node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeRecording {
    /// Node ID (unique identifier)
    pub node_id: String,
    /// Node name
    pub node_name: String,
    /// Recording session name
    pub session_name: String,
    /// When recording started
    pub started_at: u64,
    /// When recording ended
    pub ended_at: Option<u64>,
    /// First tick recorded
    pub first_tick: u64,
    /// Last tick recorded
    pub last_tick: u64,
    /// All recorded tick snapshots
    pub snapshots: Vec<NodeTickSnapshot>,
    /// Node configuration at recording time
    pub config: Option<String>,
}

impl NodeRecording {
    pub fn new(node_name: &str, node_id: &str, session_name: &str) -> Self {
        Self {
            node_id: node_id.to_string(),
            node_name: node_name.to_string(),
            session_name: session_name.to_string(),
            started_at: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            ended_at: None,
            first_tick: 0,
            last_tick: 0,
            snapshots: Vec::new(),
            config: None,
        }
    }

    /// Add a tick snapshot
    pub fn add_snapshot(&mut self, snapshot: NodeTickSnapshot) {
        if self.snapshots.is_empty() {
            self.first_tick = snapshot.tick;
        }
        self.last_tick = snapshot.tick;
        self.snapshots.push(snapshot);
    }

    /// Get snapshot for a specific tick
    pub fn get_snapshot(&self, tick: u64) -> Option<&NodeTickSnapshot> {
        self.snapshots.iter().find(|s| s.tick == tick)
    }

    /// Get snapshots in a tick range
    pub fn get_snapshots_range(&self, start_tick: u64, end_tick: u64) -> Vec<&NodeTickSnapshot> {
        self.snapshots
            .iter()
            .filter(|s| s.tick >= start_tick && s.tick <= end_tick)
            .collect()
    }

    /// Mark recording as ended
    pub fn finish(&mut self) {
        self.ended_at = Some(
            SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
        );
    }

    /// Get total number of snapshots
    pub fn snapshot_count(&self) -> usize {
        self.snapshots.len()
    }

    /// Get estimated size in bytes
    pub fn estimated_size(&self) -> usize {
        self.snapshots
            .iter()
            .map(|s| {
                s.inputs.values().map(|v| v.len()).sum::<usize>()
                    + s.outputs.values().map(|v| v.len()).sum::<usize>()
                    + s.state.as_ref().map(|v| v.len()).unwrap_or(0)
                    + 100 // Overhead estimate
            })
            .sum()
    }

    /// Save to file
    pub fn save(&self, path: &PathBuf) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        let file = File::create(path)?;
        let writer = BufWriter::new(file);

        // Use bincode for efficient serialization
        bincode::serialize_into(writer, self).map_err(|e| std::io::Error::other(e.to_string()))?;

        Ok(())
    }

    /// Load from file
    pub fn load(path: &PathBuf) -> std::io::Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        bincode::deserialize_from(reader).map_err(|e| std::io::Error::other(e.to_string()))
    }
}

/// Recording of the entire scheduler/system
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SchedulerRecording {
    /// Scheduler ID
    pub scheduler_id: String,
    /// Session name
    pub session_name: String,
    /// When recording started
    pub started_at: u64,
    /// When recording ended
    pub ended_at: Option<u64>,
    /// Total ticks recorded
    pub total_ticks: u64,
    /// Node recordings (node_id -> file path relative to session dir)
    pub node_recordings: HashMap<String, String>,
    /// Execution order per tick (for determinism)
    pub execution_order: Vec<Vec<String>>,
    /// Scheduler configuration at recording time
    pub config: Option<String>,
}

impl SchedulerRecording {
    pub fn new(scheduler_id: &str, session_name: &str) -> Self {
        Self {
            scheduler_id: scheduler_id.to_string(),
            session_name: session_name.to_string(),
            started_at: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            ended_at: None,
            total_ticks: 0,
            node_recordings: HashMap::new(),
            execution_order: Vec::new(),
            config: None,
        }
    }

    /// Register a node recording
    pub fn add_node_recording(&mut self, node_id: &str, relative_path: &str) {
        self.node_recordings
            .insert(node_id.to_string(), relative_path.to_string());
    }

    /// Record execution order for a tick
    pub fn record_execution_order(&mut self, order: Vec<String>) {
        self.execution_order.push(order);
        self.total_ticks += 1;
    }

    /// Mark recording as ended
    pub fn finish(&mut self) {
        self.ended_at = Some(
            SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
        );
    }

    /// Save to file
    pub fn save(&self, path: &PathBuf) -> std::io::Result<()> {
        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        let file = File::create(path)?;
        let writer = BufWriter::new(file);

        bincode::serialize_into(writer, self).map_err(|e| std::io::Error::other(e.to_string()))?;

        Ok(())
    }

    /// Load from file
    pub fn load(path: &PathBuf) -> std::io::Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);

        bincode::deserialize_from(reader).map_err(|e| std::io::Error::other(e.to_string()))
    }
}

/// Active recorder for a node
pub struct NodeRecorder {
    recording: NodeRecording,
    config: RecordingConfig,
    current_snapshot: Option<NodeTickSnapshot>,
    enabled: bool,
}

impl NodeRecorder {
    pub fn new(node_name: &str, node_id: &str, config: RecordingConfig) -> Self {
        Self {
            recording: NodeRecording::new(node_name, node_id, &config.session_name),
            config,
            current_snapshot: None,
            enabled: true,
        }
    }

    /// Start recording a new tick
    pub fn begin_tick(&mut self, tick: u64) {
        if !self.enabled {
            return;
        }

        // Check recording interval
        if !tick.is_multiple_of(self.config.interval) {
            self.current_snapshot = None;
            return;
        }

        self.current_snapshot = Some(NodeTickSnapshot::new(tick));
    }

    /// Record an input received
    pub fn record_input(&mut self, topic: &str, data: Vec<u8>) {
        if let Some(ref mut snapshot) = self.current_snapshot {
            snapshot.inputs.insert(topic.to_string(), data);
        }
    }

    /// Record an output produced
    pub fn record_output(&mut self, topic: &str, data: Vec<u8>) {
        if let Some(ref mut snapshot) = self.current_snapshot {
            snapshot.outputs.insert(topic.to_string(), data);
        }
    }

    /// Record internal state
    pub fn record_state(&mut self, state: Vec<u8>) {
        if let Some(ref mut snapshot) = self.current_snapshot {
            snapshot.state = Some(state);
        }
    }

    /// Finish recording the current tick
    pub fn end_tick(&mut self, duration_ns: u64) {
        if let Some(mut snapshot) = self.current_snapshot.take() {
            snapshot.duration_ns = duration_ns;
            self.recording.add_snapshot(snapshot);
        }
    }

    /// Check if we should stop (size limit reached)
    pub fn should_stop(&self) -> bool {
        self.recording.estimated_size() >= self.config.max_size
    }

    /// Finish and save the recording
    pub fn finish(&mut self) -> std::io::Result<PathBuf> {
        self.recording.finish();
        self.enabled = false;

        let path = self
            .config
            .node_path(&self.recording.node_name, &self.recording.node_id);
        self.recording.save(&path)?;

        Ok(path)
    }

    /// Get the current recording (for inspection)
    pub fn recording(&self) -> &NodeRecording {
        &self.recording
    }

    /// Enable/disable recording
    pub fn set_enabled(&mut self, enabled: bool) {
        self.enabled = enabled;
    }
}

/// Replayer for a node recording
pub struct NodeReplayer {
    recording: NodeRecording,
    current_index: usize,
    current_tick: u64,
}

impl NodeReplayer {
    /// Load a recording from file
    pub fn load(path: &PathBuf) -> std::io::Result<Self> {
        let recording = NodeRecording::load(path)?;
        Ok(Self {
            recording,
            current_index: 0,
            current_tick: 0,
        })
    }

    /// Load from a recording struct
    pub fn from_recording(recording: NodeRecording) -> Self {
        Self {
            recording,
            current_index: 0,
            current_tick: 0,
        }
    }

    /// Get the snapshot for the current tick
    pub fn current_snapshot(&self) -> Option<&NodeTickSnapshot> {
        self.recording.snapshots.get(self.current_index)
    }

    /// Get outputs for the current tick
    pub fn get_outputs(&self) -> Option<&HashMap<String, Vec<u8>>> {
        self.current_snapshot().map(|s| &s.outputs)
    }

    /// Get a specific output for the current tick
    pub fn get_output(&self, topic: &str) -> Option<&Vec<u8>> {
        self.current_snapshot().and_then(|s| s.outputs.get(topic))
    }

    /// Advance to the next tick
    pub fn advance(&mut self) -> bool {
        if self.current_index + 1 < self.recording.snapshots.len() {
            self.current_index += 1;
            if let Some(snapshot) = self.recording.snapshots.get(self.current_index) {
                self.current_tick = snapshot.tick;
            }
            true
        } else {
            false
        }
    }

    /// Jump to a specific tick
    pub fn seek(&mut self, tick: u64) -> bool {
        for (i, snapshot) in self.recording.snapshots.iter().enumerate() {
            if snapshot.tick >= tick {
                self.current_index = i;
                self.current_tick = snapshot.tick;
                return true;
            }
        }
        false
    }

    /// Reset to the beginning
    pub fn reset(&mut self) {
        self.current_index = 0;
        self.current_tick = self.recording.first_tick;
    }

    /// Check if replay is finished
    pub fn is_finished(&self) -> bool {
        self.current_index >= self.recording.snapshots.len()
    }

    /// Get the recording
    pub fn recording(&self) -> &NodeRecording {
        &self.recording
    }

    /// Get current tick number
    pub fn current_tick(&self) -> u64 {
        self.current_tick
    }

    /// Get total ticks in recording
    pub fn total_ticks(&self) -> usize {
        self.recording.snapshots.len()
    }
}

/// Manager for session discovery
pub struct RecordingManager {
    base_dir: PathBuf,
}

impl RecordingManager {
    pub fn new() -> Self {
        let base_dir = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(RECORDINGS_DIR);

        Self { base_dir }
    }

    pub fn with_base_dir(base_dir: PathBuf) -> Self {
        Self { base_dir }
    }

    /// List all recording sessions
    pub fn list_sessions(&self) -> std::io::Result<Vec<String>> {
        let mut sessions = Vec::new();

        if self.base_dir.exists() {
            for entry in fs::read_dir(&self.base_dir)? {
                let entry = entry?;
                if entry.file_type()?.is_dir() {
                    if let Some(name) = entry.file_name().to_str() {
                        sessions.push(name.to_string());
                    }
                }
            }
        }

        Ok(sessions)
    }

    /// Get all recordings in a session
    pub fn get_session_recordings(&self, session: &str) -> std::io::Result<Vec<PathBuf>> {
        let session_dir = self.base_dir.join(session);
        let mut recordings = Vec::new();

        if session_dir.exists() {
            for entry in fs::read_dir(&session_dir)? {
                let entry = entry?;
                let path = entry.path();
                if path
                    .extension()
                    .map(|e| e == RECORDING_EXT)
                    .unwrap_or(false)
                {
                    recordings.push(path);
                }
            }
        }

        Ok(recordings)
    }

    /// Delete a session and all its recordings
    pub fn delete_session(&self, session: &str) -> std::io::Result<()> {
        let session_dir = self.base_dir.join(session);
        if session_dir.exists() {
            fs::remove_dir_all(session_dir)?;
        }
        Ok(())
    }

    /// Get total size of recordings
    pub fn total_size(&self) -> std::io::Result<u64> {
        let mut total = 0;

        if self.base_dir.exists() {
            for session in self.list_sessions()? {
                for path in self.get_session_recordings(&session)? {
                    if let Ok(metadata) = fs::metadata(&path) {
                        total += metadata.len();
                    }
                }
            }
        }

        Ok(total)
    }
}

impl Default for RecordingManager {
    fn default() -> Self {
        Self::new()
    }
}

/// Compare two recordings for differences
pub fn diff_recordings(
    recording1: &NodeRecording,
    recording2: &NodeRecording,
) -> Vec<RecordingDiff> {
    let mut diffs = Vec::new();

    // Find common tick range
    let start = recording1.first_tick.max(recording2.first_tick);
    let end = recording1.last_tick.min(recording2.last_tick);

    for tick in start..=end {
        let snap1 = recording1.get_snapshot(tick);
        let snap2 = recording2.get_snapshot(tick);

        match (snap1, snap2) {
            (Some(s1), Some(s2)) => {
                // Compare outputs
                for (topic, data1) in &s1.outputs {
                    if let Some(data2) = s2.outputs.get(topic) {
                        if data1 != data2 {
                            diffs.push(RecordingDiff::OutputDifference {
                                tick,
                                topic: topic.clone(),
                                recording1_size: data1.len(),
                                recording2_size: data2.len(),
                            });
                        }
                    } else {
                        diffs.push(RecordingDiff::MissingOutput {
                            tick,
                            topic: topic.clone(),
                            in_recording: 1,
                        });
                    }
                }

                // Check for outputs only in recording2
                for topic in s2.outputs.keys() {
                    if !s1.outputs.contains_key(topic) {
                        diffs.push(RecordingDiff::MissingOutput {
                            tick,
                            topic: topic.clone(),
                            in_recording: 2,
                        });
                    }
                }
            }
            (Some(_), None) => {
                diffs.push(RecordingDiff::MissingTick {
                    tick,
                    in_recording: 2,
                });
            }
            (None, Some(_)) => {
                diffs.push(RecordingDiff::MissingTick {
                    tick,
                    in_recording: 1,
                });
            }
            (None, None) => {}
        }
    }

    diffs
}

/// Difference between two recordings
#[derive(Debug, Clone)]
pub enum RecordingDiff {
    /// Output data differs at this tick
    OutputDifference {
        tick: u64,
        topic: String,
        recording1_size: usize,
        recording2_size: usize,
    },
    /// Output missing in one recording
    MissingOutput {
        tick: u64,
        topic: String,
        in_recording: u8,
    },
    /// Tick missing in one recording
    MissingTick { tick: u64, in_recording: u8 },
}

// ============================================================================
// ReplayNode - Node wrapper for replaying recordings
// ============================================================================

use crate::core::Node;

/// A node that replays recorded data instead of executing real logic.
///
/// This wrapper allows mixing live nodes with recorded data for debugging.
pub struct ReplayNode {
    node_name: String,
    node_id: String,
    tick_count: u64,
}

impl ReplayNode {
    /// Create a new replay node.
    pub fn new(node_name: String, node_id: String) -> Self {
        Self {
            node_name,
            node_id,
            tick_count: 0,
        }
    }

    /// Get the node ID
    pub fn node_id(&self) -> &str {
        &self.node_id
    }
}

impl Node for ReplayNode {
    fn name(&self) -> &str {
        &self.node_name
    }

    fn tick(&mut self) {
        // The actual replay logic is handled by the Scheduler,
        // which looks up the NodeReplayer and publishes recorded outputs.
        // This tick just tracks the count for logging.
        self.tick_count += 1;
    }

    // Use default implementations for publishers and subscribers
    // which return empty Vec - replay nodes don't declare topics
}

// ============================================================================
// Advanced Debugging: Breakpoints, Stepping, Watch Expressions
// ============================================================================

/// Breakpoint condition for debugging
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum BreakpointCondition {
    /// Break at a specific tick
    AtTick(u64),
    /// Break when tick equals value
    TickEquals(u64),
    /// Break when a specific topic has data
    TopicHasData(String),
    /// Break when output data matches pattern (simple byte equality)
    OutputMatches { topic: String, pattern: Vec<u8> },
    /// Break on node error (output contains "error" in topic name)
    OnError,
    /// Break after N ticks from current position
    AfterTicks(u64),
    /// Custom expression (evaluated as simple field access)
    Expression(WatchExpression),
}

/// Watch expression for monitoring values during replay
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WatchExpression {
    /// Unique ID for this watch
    pub id: String,
    /// Human-readable name
    pub name: String,
    /// Topic to watch
    pub topic: String,
    /// Whether to watch inputs or outputs
    pub watch_type: WatchType,
    /// Optional byte offset to extract value
    pub byte_offset: Option<usize>,
    /// Optional byte length to extract
    pub byte_length: Option<usize>,
}

impl WatchExpression {
    pub fn new(id: &str, name: &str, topic: &str, watch_type: WatchType) -> Self {
        Self {
            id: id.to_string(),
            name: name.to_string(),
            topic: topic.to_string(),
            watch_type,
            byte_offset: None,
            byte_length: None,
        }
    }

    /// Create a watch for output data
    pub fn output(id: &str, name: &str, topic: &str) -> Self {
        Self::new(id, name, topic, WatchType::Output)
    }

    /// Create a watch for input data
    pub fn input(id: &str, name: &str, topic: &str) -> Self {
        Self::new(id, name, topic, WatchType::Input)
    }

    /// Set byte range to extract
    pub fn with_range(mut self, offset: usize, length: usize) -> Self {
        self.byte_offset = Some(offset);
        self.byte_length = Some(length);
        self
    }

    /// Evaluate this watch expression against a snapshot
    pub fn evaluate(&self, snapshot: &NodeTickSnapshot) -> Option<WatchValue> {
        let data = match self.watch_type {
            WatchType::Input => snapshot.inputs.get(&self.topic),
            WatchType::Output => snapshot.outputs.get(&self.topic),
        }?;

        let value = match (self.byte_offset, self.byte_length) {
            (Some(offset), Some(length)) => {
                if offset + length <= data.len() {
                    data[offset..offset + length].to_vec()
                } else {
                    return None;
                }
            }
            (Some(offset), None) => {
                if offset < data.len() {
                    data[offset..].to_vec()
                } else {
                    return None;
                }
            }
            _ => data.clone(),
        };

        Some(WatchValue {
            expression_id: self.id.clone(),
            tick: snapshot.tick,
            raw_bytes: value.clone(),
            display_value: format_bytes_as_value(&value),
        })
    }
}

/// Type of data to watch
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum WatchType {
    Input,
    Output,
}

/// Result of evaluating a watch expression
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WatchValue {
    /// The expression that produced this value
    pub expression_id: String,
    /// Tick at which this was evaluated
    pub tick: u64,
    /// Raw bytes
    pub raw_bytes: Vec<u8>,
    /// Human-readable display value
    pub display_value: String,
}

/// Format bytes as a displayable value
fn format_bytes_as_value(bytes: &[u8]) -> String {
    // Try to interpret as various types
    match bytes.len() {
        1 => format!("u8: {}, i8: {}", bytes[0], bytes[0] as i8),
        2 => {
            let u16_val = u16::from_le_bytes([bytes[0], bytes[1]]);
            let i16_val = i16::from_le_bytes([bytes[0], bytes[1]]);
            format!("u16: {}, i16: {}", u16_val, i16_val)
        }
        4 => {
            let u32_val = u32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            let i32_val = i32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            let f32_val = f32::from_le_bytes([bytes[0], bytes[1], bytes[2], bytes[3]]);
            format!("u32: {}, i32: {}, f32: {:.6}", u32_val, i32_val, f32_val)
        }
        8 => {
            let u64_val = u64::from_le_bytes(bytes[0..8].try_into().unwrap_or([0; 8]));
            let i64_val = i64::from_le_bytes(bytes[0..8].try_into().unwrap_or([0; 8]));
            let f64_val = f64::from_le_bytes(bytes[0..8].try_into().unwrap_or([0; 8]));
            format!("u64: {}, i64: {}, f64: {:.6}", u64_val, i64_val, f64_val)
        }
        _ if bytes.len() <= 32 => {
            // Try as UTF-8 string
            if let Ok(s) = std::str::from_utf8(bytes) {
                if s.chars().all(|c| !c.is_control() || c == '\n' || c == '\t') {
                    return format!("str: \"{}\"", s);
                }
            }
            format!("bytes[{}]: {:02x?}", bytes.len(), bytes)
        }
        _ => format!("bytes[{}]: {:02x?}...", bytes.len(), &bytes[..32]),
    }
}

/// Debugger state
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DebuggerState {
    /// Running normally
    Running,
    /// Paused at a breakpoint
    Paused,
    /// Stepping forward one tick at a time
    StepForward,
    /// Stepping backward one tick at a time
    StepBackward,
    /// Stopped (finished or error)
    Stopped,
}

/// A breakpoint with its metadata
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Breakpoint {
    /// Unique ID
    pub id: u32,
    /// The condition that triggers this breakpoint
    pub condition: BreakpointCondition,
    /// Whether this breakpoint is enabled
    pub enabled: bool,
    /// How many times this breakpoint has been hit
    pub hit_count: u32,
    /// Optional name for this breakpoint
    pub name: Option<String>,
}

impl Breakpoint {
    pub fn new(id: u32, condition: BreakpointCondition) -> Self {
        Self {
            id,
            condition,
            enabled: true,
            hit_count: 0,
            name: None,
        }
    }

    pub fn with_name(mut self, name: &str) -> Self {
        self.name = Some(name.to_string());
        self
    }

    /// Check if this breakpoint should trigger
    pub fn should_trigger(&self, snapshot: &NodeTickSnapshot, current_tick: u64) -> bool {
        if !self.enabled {
            return false;
        }

        match &self.condition {
            BreakpointCondition::AtTick(tick) => snapshot.tick == *tick,
            BreakpointCondition::TickEquals(tick) => snapshot.tick == *tick,
            BreakpointCondition::TopicHasData(topic) => {
                snapshot.inputs.contains_key(topic) || snapshot.outputs.contains_key(topic)
            }
            BreakpointCondition::OutputMatches { topic, pattern } => snapshot
                .outputs
                .get(topic)
                .map(|d| d == pattern)
                .unwrap_or(false),
            BreakpointCondition::OnError => {
                // Check if any topic name contains "error"
                snapshot
                    .outputs
                    .keys()
                    .any(|k| k.to_lowercase().contains("error"))
                    || snapshot
                        .inputs
                        .keys()
                        .any(|k| k.to_lowercase().contains("error"))
            }
            BreakpointCondition::AfterTicks(n) => snapshot.tick >= current_tick + n,
            BreakpointCondition::Expression(expr) => {
                // Check if the expression evaluates to non-empty data
                expr.evaluate(snapshot).is_some()
            }
        }
    }
}

/// Event emitted by the debugger
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum DebugEvent {
    /// Breakpoint was hit
    BreakpointHit { breakpoint_id: u32, tick: u64 },
    /// Watch value changed
    WatchValueChanged {
        watch_id: String,
        old_value: Option<WatchValue>,
        new_value: WatchValue,
    },
    /// Replay position changed
    PositionChanged { tick: u64, index: usize },
    /// Debugger state changed
    StateChanged {
        old_state: DebuggerState,
        new_state: DebuggerState,
    },
    /// Replay finished
    Finished { total_ticks: u64 },
    /// Error occurred
    Error { message: String },
}

/// Advanced debugger for replay sessions
pub struct ReplayDebugger {
    /// The replayer being debugged
    replayer: NodeReplayer,
    /// Current debugger state
    state: DebuggerState,
    /// All breakpoints
    breakpoints: Vec<Breakpoint>,
    /// Watch expressions
    watches: Vec<WatchExpression>,
    /// Last evaluated watch values (for change detection)
    last_watch_values: HashMap<String, WatchValue>,
    /// Event history
    events: Vec<DebugEvent>,
    /// Next breakpoint ID
    next_breakpoint_id: u32,
    /// Max events to keep in history
    max_events: usize,
}

impl ReplayDebugger {
    /// Create a new debugger for a replayer
    pub fn new(replayer: NodeReplayer) -> Self {
        Self {
            replayer,
            state: DebuggerState::Paused,
            breakpoints: Vec::new(),
            watches: Vec::new(),
            last_watch_values: HashMap::new(),
            events: Vec::new(),
            next_breakpoint_id: 1,
            max_events: 1000,
        }
    }

    /// Load a recording and create a debugger
    pub fn load(path: &PathBuf) -> std::io::Result<Self> {
        let replayer = NodeReplayer::load(path)?;
        Ok(Self::new(replayer))
    }

    /// Get the current debugger state
    pub fn state(&self) -> DebuggerState {
        self.state
    }

    /// Get the current tick
    pub fn current_tick(&self) -> u64 {
        self.replayer.current_tick()
    }

    /// Get the current snapshot
    pub fn current_snapshot(&self) -> Option<&NodeTickSnapshot> {
        self.replayer.current_snapshot()
    }

    /// Get the recording being debugged
    pub fn recording(&self) -> &NodeRecording {
        self.replayer.recording()
    }

    // --- Breakpoint Management ---

    /// Add a breakpoint at a specific tick
    pub fn add_breakpoint_at_tick(&mut self, tick: u64) -> u32 {
        self.add_breakpoint(BreakpointCondition::AtTick(tick))
    }

    /// Add a breakpoint with a custom condition
    pub fn add_breakpoint(&mut self, condition: BreakpointCondition) -> u32 {
        let id = self.next_breakpoint_id;
        self.next_breakpoint_id += 1;
        self.breakpoints.push(Breakpoint::new(id, condition));
        id
    }

    /// Add a named breakpoint
    pub fn add_named_breakpoint(&mut self, name: &str, condition: BreakpointCondition) -> u32 {
        let id = self.next_breakpoint_id;
        self.next_breakpoint_id += 1;
        self.breakpoints
            .push(Breakpoint::new(id, condition).with_name(name));
        id
    }

    /// Remove a breakpoint
    pub fn remove_breakpoint(&mut self, id: u32) -> bool {
        if let Some(pos) = self.breakpoints.iter().position(|b| b.id == id) {
            self.breakpoints.remove(pos);
            true
        } else {
            false
        }
    }

    /// Enable/disable a breakpoint
    pub fn set_breakpoint_enabled(&mut self, id: u32, enabled: bool) -> bool {
        if let Some(bp) = self.breakpoints.iter_mut().find(|b| b.id == id) {
            bp.enabled = enabled;
            true
        } else {
            false
        }
    }

    /// Get all breakpoints
    pub fn breakpoints(&self) -> &[Breakpoint] {
        &self.breakpoints
    }

    /// Clear all breakpoints
    pub fn clear_breakpoints(&mut self) {
        self.breakpoints.clear();
    }

    // --- Watch Expressions ---

    /// Add a watch expression
    pub fn add_watch(&mut self, watch: WatchExpression) {
        self.watches.push(watch);
    }

    /// Remove a watch expression
    pub fn remove_watch(&mut self, id: &str) -> bool {
        if let Some(pos) = self.watches.iter().position(|w| w.id == id) {
            self.watches.remove(pos);
            self.last_watch_values.remove(id);
            true
        } else {
            false
        }
    }

    /// Get all watches
    pub fn watches(&self) -> &[WatchExpression] {
        &self.watches
    }

    /// Evaluate all watch expressions for the current snapshot
    pub fn evaluate_watches(&mut self) -> Vec<WatchValue> {
        // Clone the snapshot to avoid borrow conflicts
        let snapshot = match self.replayer.current_snapshot() {
            Some(s) => s.clone(),
            None => return Vec::new(),
        };

        // First pass: evaluate all watches and collect changes
        let mut values = Vec::new();
        let mut events_to_emit = Vec::new();

        for watch in &self.watches {
            if let Some(value) = watch.evaluate(&snapshot) {
                // Check for change
                let changed = self
                    .last_watch_values
                    .get(&watch.id)
                    .map(|old| old.raw_bytes != value.raw_bytes)
                    .unwrap_or(true);

                if changed {
                    let old_value = self.last_watch_values.get(&watch.id).cloned();
                    events_to_emit.push(DebugEvent::WatchValueChanged {
                        watch_id: watch.id.clone(),
                        old_value,
                        new_value: value.clone(),
                    });
                }

                self.last_watch_values
                    .insert(watch.id.clone(), value.clone());
                values.push(value);
            }
        }

        // Second pass: emit events (now we can borrow self mutably)
        for event in events_to_emit {
            self.emit_event(event);
        }

        values
    }

    // --- Stepping Controls ---

    /// Continue execution until a breakpoint is hit
    pub fn continue_execution(&mut self) -> Option<&DebugEvent> {
        let old_state = self.state;
        self.state = DebuggerState::Running;
        self.emit_event(DebugEvent::StateChanged {
            old_state,
            new_state: self.state,
        });

        while self.state == DebuggerState::Running {
            if !self.step_internal(true) {
                break;
            }
        }

        self.events.last()
    }

    /// Step forward one tick
    pub fn step_forward(&mut self) -> bool {
        let old_state = self.state;
        self.state = DebuggerState::StepForward;
        if old_state != self.state {
            self.emit_event(DebugEvent::StateChanged {
                old_state,
                new_state: self.state,
            });
        }

        let result = self.step_internal(false);
        self.state = DebuggerState::Paused;
        result
    }

    /// Step backward one tick
    pub fn step_backward(&mut self) -> bool {
        let old_state = self.state;
        self.state = DebuggerState::StepBackward;
        if old_state != self.state {
            self.emit_event(DebugEvent::StateChanged {
                old_state,
                new_state: self.state,
            });
        }

        // Find the previous snapshot
        let recording = self.replayer.recording();
        let current_tick = self.replayer.current_tick();

        // Find the previous tick in the recording
        let mut prev_tick = None;
        for snapshot in &recording.snapshots {
            if snapshot.tick < current_tick {
                prev_tick = Some(snapshot.tick);
            } else {
                break;
            }
        }

        if let Some(tick) = prev_tick {
            self.replayer.seek(tick);
            self.evaluate_watches();
            self.emit_event(DebugEvent::PositionChanged {
                tick,
                index: self.replayer.current_tick() as usize,
            });
            self.state = DebuggerState::Paused;
            true
        } else {
            self.state = DebuggerState::Paused;
            false
        }
    }

    /// Pause execution
    pub fn pause(&mut self) {
        let old_state = self.state;
        self.state = DebuggerState::Paused;
        if old_state != self.state {
            self.emit_event(DebugEvent::StateChanged {
                old_state,
                new_state: self.state,
            });
        }
    }

    /// Stop the debugger
    pub fn stop(&mut self) {
        let old_state = self.state;
        self.state = DebuggerState::Stopped;
        if old_state != self.state {
            self.emit_event(DebugEvent::StateChanged {
                old_state,
                new_state: self.state,
            });
        }
    }

    /// Seek to a specific tick
    pub fn seek(&mut self, tick: u64) -> bool {
        if self.replayer.seek(tick) {
            self.evaluate_watches();
            self.emit_event(DebugEvent::PositionChanged {
                tick: self.replayer.current_tick(),
                index: 0,
            });
            true
        } else {
            false
        }
    }

    /// Reset to the beginning
    pub fn reset(&mut self) {
        self.replayer.reset();
        self.last_watch_values.clear();
        self.state = DebuggerState::Paused;
        self.emit_event(DebugEvent::PositionChanged {
            tick: self.replayer.current_tick(),
            index: 0,
        });
    }

    // --- Internal ---

    fn step_internal(&mut self, check_breakpoints: bool) -> bool {
        let current_tick = self.replayer.current_tick();

        // Check if we're at the end
        if self.replayer.is_finished() {
            self.state = DebuggerState::Stopped;
            self.emit_event(DebugEvent::Finished {
                total_ticks: current_tick,
            });
            return false;
        }

        // Advance
        if !self.replayer.advance() {
            self.state = DebuggerState::Stopped;
            self.emit_event(DebugEvent::Finished {
                total_ticks: current_tick,
            });
            return false;
        }

        // Emit position change
        self.emit_event(DebugEvent::PositionChanged {
            tick: self.replayer.current_tick(),
            index: 0,
        });

        // Evaluate watches
        self.evaluate_watches();

        // Check breakpoints
        if check_breakpoints {
            if let Some(snapshot) = self.replayer.current_snapshot() {
                for bp in &mut self.breakpoints {
                    if bp.should_trigger(snapshot, current_tick) {
                        bp.hit_count += 1;
                        self.state = DebuggerState::Paused;
                        // Need to emit event outside the loop due to borrow
                    }
                }

                // Check if any breakpoint was hit
                let hit_bp = self
                    .breakpoints
                    .iter()
                    .find(|bp| bp.enabled && bp.should_trigger(snapshot, current_tick))
                    .map(|bp| bp.id);

                if let Some(bp_id) = hit_bp {
                    self.emit_event(DebugEvent::BreakpointHit {
                        breakpoint_id: bp_id,
                        tick: self.replayer.current_tick(),
                    });
                }
            }
        }

        true
    }

    fn emit_event(&mut self, event: DebugEvent) {
        self.events.push(event);
        if self.events.len() > self.max_events {
            self.events.remove(0);
        }
    }

    /// Get recent events
    pub fn events(&self) -> &[DebugEvent] {
        &self.events
    }

    /// Get the last N events
    pub fn recent_events(&self, n: usize) -> &[DebugEvent] {
        let start = self.events.len().saturating_sub(n);
        &self.events[start..]
    }

    /// Clear event history
    pub fn clear_events(&mut self) {
        self.events.clear();
    }

    /// Get the underlying replayer
    pub fn replayer(&self) -> &NodeReplayer {
        &self.replayer
    }

    /// Get mutable access to the underlying replayer
    pub fn replayer_mut(&mut self) -> &mut NodeReplayer {
        &mut self.replayer
    }
}

// ============================================================================
// Debug Session State (for serialization/resumption)
// ============================================================================

/// Serializable debug session state
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DebugSessionState {
    /// Path to the recording being debugged
    pub recording_path: PathBuf,
    /// Current tick position
    pub current_tick: u64,
    /// All breakpoints
    pub breakpoints: Vec<Breakpoint>,
    /// All watch expressions
    pub watches: Vec<WatchExpression>,
    /// Session name
    pub session_name: String,
    /// When this session was created
    pub created_at: u64,
    /// When this session was last updated
    pub updated_at: u64,
}

impl DebugSessionState {
    /// Create a new debug session state
    pub fn new(recording_path: PathBuf, session_name: &str) -> Self {
        let now = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();

        Self {
            recording_path,
            current_tick: 0,
            breakpoints: Vec::new(),
            watches: Vec::new(),
            session_name: session_name.to_string(),
            created_at: now,
            updated_at: now,
        }
    }

    /// Save to file
    pub fn save(&self, path: &PathBuf) -> std::io::Result<()> {
        let json =
            serde_json::to_string_pretty(self).map_err(|e| std::io::Error::other(e.to_string()))?;
        fs::write(path, json)
    }

    /// Load from file
    pub fn load(path: &PathBuf) -> std::io::Result<Self> {
        let json = fs::read_to_string(path)?;
        serde_json::from_str(&json).map_err(|e| std::io::Error::other(e.to_string()))
    }

    /// Create a debugger from this session state
    pub fn create_debugger(&self) -> std::io::Result<ReplayDebugger> {
        let mut debugger = ReplayDebugger::load(&self.recording_path)?;

        // Restore breakpoints
        for bp in &self.breakpoints {
            debugger.breakpoints.push(bp.clone());
            if bp.id >= debugger.next_breakpoint_id {
                debugger.next_breakpoint_id = bp.id + 1;
            }
        }

        // Restore watches
        for watch in &self.watches {
            debugger.watches.push(watch.clone());
        }

        // Seek to saved position
        debugger.seek(self.current_tick);

        Ok(debugger)
    }

    /// Update from debugger state
    pub fn update_from_debugger(&mut self, debugger: &ReplayDebugger) {
        self.current_tick = debugger.current_tick();
        self.breakpoints = debugger.breakpoints.clone();
        self.watches = debugger.watches.clone();
        self.updated_at = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs();
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::tempdir;

    #[test]
    fn test_node_recording() {
        let mut recording = NodeRecording::new("test_node", "abc123", "test_session");

        let snapshot1 = NodeTickSnapshot::new(0)
            .with_input("sensor", vec![1, 2, 3])
            .with_output("motor", vec![4, 5, 6]);

        let snapshot2 = NodeTickSnapshot::new(1)
            .with_input("sensor", vec![7, 8, 9])
            .with_output("motor", vec![10, 11, 12]);

        recording.add_snapshot(snapshot1);
        recording.add_snapshot(snapshot2);

        assert_eq!(recording.first_tick, 0);
        assert_eq!(recording.last_tick, 1);
        assert_eq!(recording.snapshot_count(), 2);

        let snap = recording.get_snapshot(1).unwrap();
        assert_eq!(snap.inputs.get("sensor").unwrap(), &vec![7, 8, 9]);
    }

    #[test]
    fn test_recording_save_load() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("test.horus");

        let mut recording = NodeRecording::new("test_node", "abc123", "test_session");
        recording.add_snapshot(NodeTickSnapshot::new(0).with_output("out", vec![1, 2, 3]));
        recording.finish();

        recording.save(&path).unwrap();

        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.node_name, "test_node");
        assert_eq!(loaded.snapshot_count(), 1);
    }

    #[test]
    fn test_node_recorder() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "test".to_string(),
            base_dir: dir.path().to_path_buf(),
            ..Default::default()
        };

        let mut recorder = NodeRecorder::new("test_node", "abc123", config);

        recorder.begin_tick(0);
        recorder.record_input("sensor", vec![1, 2, 3]);
        recorder.record_output("motor", vec![4, 5, 6]);
        recorder.end_tick(1000);

        recorder.begin_tick(1);
        recorder.record_input("sensor", vec![7, 8, 9]);
        recorder.record_output("motor", vec![10, 11, 12]);
        recorder.end_tick(2000);

        assert_eq!(recorder.recording().snapshot_count(), 2);

        let path = recorder.finish().unwrap();
        assert!(path.exists());
    }

    #[test]
    fn test_node_replayer() {
        let mut recording = NodeRecording::new("test_node", "abc123", "test_session");
        recording.add_snapshot(NodeTickSnapshot::new(0).with_output("motor", vec![1, 2, 3]));
        recording.add_snapshot(NodeTickSnapshot::new(1).with_output("motor", vec![4, 5, 6]));
        recording.add_snapshot(NodeTickSnapshot::new(2).with_output("motor", vec![7, 8, 9]));

        let mut replayer = NodeReplayer::from_recording(recording);

        assert_eq!(replayer.current_tick(), 0);
        assert_eq!(replayer.get_output("motor").unwrap(), &vec![1, 2, 3]);

        replayer.advance();
        assert_eq!(replayer.current_tick(), 1);
        assert_eq!(replayer.get_output("motor").unwrap(), &vec![4, 5, 6]);

        replayer.seek(2);
        assert_eq!(replayer.current_tick(), 2);

        replayer.reset();
        assert_eq!(replayer.current_tick(), 0);
    }

    #[test]
    fn test_recording_diff() {
        let mut recording1 = NodeRecording::new("node", "1", "session");
        let mut recording2 = NodeRecording::new("node", "2", "session");

        // Same tick 0
        recording1.add_snapshot(NodeTickSnapshot::new(0).with_output("out", vec![1, 2, 3]));
        recording2.add_snapshot(NodeTickSnapshot::new(0).with_output("out", vec![1, 2, 3]));

        // Different tick 1
        recording1.add_snapshot(NodeTickSnapshot::new(1).with_output("out", vec![4, 5, 6]));
        recording2.add_snapshot(NodeTickSnapshot::new(1).with_output("out", vec![7, 8, 9]));

        let diffs = diff_recordings(&recording1, &recording2);
        assert_eq!(diffs.len(), 1);

        match &diffs[0] {
            RecordingDiff::OutputDifference { tick, topic, .. } => {
                assert_eq!(*tick, 1);
                assert_eq!(topic, "out");
            }
            _ => panic!("Expected OutputDifference"),
        }
    }

    #[test]
    fn test_recording_interval() {
        let config = RecordingConfig {
            session_name: "test".to_string(),
            base_dir: PathBuf::from("/tmp"),
            interval: 2, // Record every 2 ticks
            ..Default::default()
        };

        let mut recorder = NodeRecorder::new("test_node", "abc123", config);

        recorder.begin_tick(0);
        recorder.record_output("out", vec![1]);
        recorder.end_tick(100);

        recorder.begin_tick(1); // Should be skipped
        recorder.record_output("out", vec![2]);
        recorder.end_tick(100);

        recorder.begin_tick(2);
        recorder.record_output("out", vec![3]);
        recorder.end_tick(100);

        assert_eq!(recorder.recording().snapshot_count(), 2); // Only ticks 0 and 2
    }
}
