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
use std::path::{Component, PathBuf};
use std::time::{Instant, SystemTime};

/// Sanitize a path component to prevent directory traversal attacks.
///
/// Strips `..`, absolute prefixes, and any OS-specific separators so the
/// resulting string is always a single, flat filename component.
fn sanitize_path_component(name: &str) -> String {
    let path = std::path::Path::new(name);
    let safe: String = path
        .components()
        .filter_map(|c| match c {
            Component::Normal(s) => s.to_str(),
            _ => None, // Drop RootDir, ParentDir, Prefix, CurDir
        })
        .next_back() // Take only the final normal component
        .unwrap_or("unnamed")
        .to_string();
    if safe.is_empty() {
        "unnamed".to_string()
    } else {
        safe
    }
}

/// Legacy relative path for recordings (only used as CWD-local fallback).
const RECORDINGS_DIR: &str = ".horus/recordings";

/// Canonical recordings base directory using the platform data dir.
///
/// Returns `horus_sys::platform::data_dir()/recordings`
/// (e.g. `~/.local/share/horus/recordings` on Linux).
fn default_recordings_dir() -> PathBuf {
    horus_sys::platform::data_dir().join("recordings")
}

/// Recording file extension
const RECORDING_EXT: &str = "horus";

/// Recording configuration
#[derive(Debug, Clone)]
pub struct RecordingConfig {
    /// Session name
    pub session_name: String,
    /// Base directory for recordings
    pub base_dir: PathBuf,
    /// Record interval (record every N ticks, 1 = every tick)
    pub interval: u64,
    /// Whether to record input values
    pub record_inputs: bool,
    /// Whether to record output values
    pub record_outputs: bool,
    /// Whether to record timing information
    pub record_timing: bool,
    /// Maximum recording size in bytes (0 = unlimited).
    /// Converted from the YAML `max_size_mb` field at config load time.
    pub max_size_bytes: u64,
    /// Enable zstd compression when saving recordings.
    pub compress: bool,
    /// Maximum number of snapshots to keep (0 = unlimited).
    /// When exceeded, oldest snapshots are evicted (ring-buffer mode).
    pub max_snapshots: usize,
}

impl Default for RecordingConfig {
    fn default() -> Self {
        let base_dir = default_recordings_dir();

        let session_name = std::env::var("HORUS_RECORD_SESSION")
            .ok()
            .filter(|s| !s.is_empty())
            .unwrap_or_else(|| {
                let now = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap_or_default();
                // Millisecond precision + process ID to avoid collisions
                format!(
                    "recording_{}.{}_p{}",
                    now.as_secs(),
                    now.subsec_millis(),
                    std::process::id()
                )
            });

        Self {
            session_name,
            base_dir,
            interval: 1,
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
            max_size_bytes: 0,
            compress: false,
            max_snapshots: 0,
        }
    }
}

impl RecordingConfig {
    /// Get the session directory.
    ///
    /// Session name is sanitized to prevent directory traversal.
    pub fn session_dir(&self) -> PathBuf {
        self.base_dir
            .join(sanitize_path_component(&self.session_name))
    }

    /// Get the path for a node recording.
    ///
    /// Path components are sanitized to prevent directory traversal.
    /// The `@` character is replaced with `_` in node names to preserve
    /// the `name@id.horus` filename convention.
    pub fn node_path(&self, node_name: &str, node_id: &str) -> PathBuf {
        let safe_name = sanitize_path_component(node_name).replace('@', "_");
        let safe_id = sanitize_path_component(node_id).replace('@', "_");
        self.session_dir()
            .join(format!("{}@{}.{}", safe_name, safe_id, RECORDING_EXT))
    }

    /// Get the path for the scheduler recording.
    ///
    /// Path components are sanitized to prevent directory traversal.
    pub fn scheduler_path(&self, scheduler_id: &str) -> PathBuf {
        let safe_id = sanitize_path_component(scheduler_id);
        self.session_dir()
            .join(format!("scheduler@{}.{}", safe_id, RECORDING_EXT))
    }
}

/// Convert YAML-based recording config to runtime recording config.
///
/// This bridges the declarative `SchedulerConfig` with the runtime
/// `RecordingConfig` used by `NodeRecorder` and `Scheduler::apply_config()`.
impl From<super::config::RecordingConfigYaml> for RecordingConfig {
    fn from(yaml: super::config::RecordingConfigYaml) -> Self {
        let base_dir = yaml
            .output_dir
            .map(PathBuf::from)
            .unwrap_or_else(default_recordings_dir);

        let session_name = yaml
            .session_name
            .or_else(|| {
                std::env::var("HORUS_RECORD_SESSION")
                    .ok()
                    .filter(|s| !s.is_empty())
            })
            .unwrap_or_else(|| {
                let now = SystemTime::now()
                    .duration_since(SystemTime::UNIX_EPOCH)
                    .unwrap_or_default();
                format!(
                    "recording_{}.{}_p{}",
                    now.as_secs(),
                    now.subsec_millis(),
                    std::process::id()
                )
            });

        Self {
            session_name,
            base_dir,
            interval: (yaml.interval as u64).max(1),
            record_inputs: yaml.record_inputs,
            record_outputs: yaml.record_outputs,
            record_timing: yaml.record_timing,
            max_size_bytes: (yaml.max_size_mb as u64).saturating_mul(1024 * 1024),
            compress: yaml.compress,
            max_snapshots: 0, // Configurable via API, not yet exposed in YAML
        }
    }
}

/// Magic bytes identifying a horus recording file.
const RECORDING_MAGIC: &[u8; 6] = b"HORUS\0";

/// Current recording format version (uncompressed).
const RECORDING_FORMAT_VERSION: u32 = 1;

/// Recording format version for zstd-compressed files.
#[cfg(feature = "recording-compression")]
const RECORDING_FORMAT_VERSION_ZSTD: u32 = 2;

/// Shared trait for recording types that support save/load/finish via bincode.
///
/// Implementors must provide access to their `ended_at` field.
pub trait Recording: Serialize + serde::de::DeserializeOwned + Sized {
    /// Mutable access to the `ended_at` timestamp.
    fn ended_at_mut(&mut self) -> &mut Option<u64>;

    /// Mark recording as ended (sets `ended_at` to current time).
    fn finish(&mut self) {
        *self.ended_at_mut() = Some(
            SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
        );
    }

    /// Save to file using bincode with a version header (uncompressed).
    ///
    /// File layout: `[6B magic "HORUS\0"][4B version LE][bincode payload]`
    fn save(&self, path: &PathBuf) -> std::io::Result<()> {
        self.save_with_compression(path, false)
    }

    /// Save to file with optional zstd compression.
    ///
    /// When `compress` is true and the `recording-compression` feature is
    /// enabled, the bincode payload is zstd-compressed (version 2).
    /// Otherwise, writes uncompressed (version 1).
    fn save_with_compression(&self, path: &PathBuf, compress: bool) -> std::io::Result<()> {
        use std::io::Write;

        if let Some(parent) = path.parent() {
            fs::create_dir_all(parent)?;
        }

        // Atomic write: write to temp file, then rename
        let tmp_path = path.with_extension("horus.tmp");
        let file = File::create(&tmp_path)?;
        let mut writer = BufWriter::new(file);

        #[cfg(feature = "recording-compression")]
        if compress {
            // Write header: magic + version 2
            writer.write_all(RECORDING_MAGIC)?;
            writer.write_all(&RECORDING_FORMAT_VERSION_ZSTD.to_le_bytes())?;

            // Compress bincode payload with zstd
            let data =
                bincode::serialize(self).map_err(|e| std::io::Error::other(e.to_string()))?;
            let compressed = zstd::encode_all(data.as_slice(), 3)
                .map_err(|e| std::io::Error::other(format!("zstd compress: {}", e)))?;
            writer.write_all(&compressed)?;
            writer.flush()?;
            drop(writer);
            fs::rename(&tmp_path, path)?;
            return Ok(());
        }

        #[cfg(not(feature = "recording-compression"))]
        let _ = compress;

        // Uncompressed: magic + version 1 + bincode
        writer.write_all(RECORDING_MAGIC)?;
        writer.write_all(&RECORDING_FORMAT_VERSION.to_le_bytes())?;
        bincode::serialize_into(&mut writer, self)
            .map_err(|e| std::io::Error::other(e.to_string()))?;
        writer.flush()?;
        drop(writer);
        fs::rename(&tmp_path, path)?;
        Ok(())
    }

    /// Load from file using bincode with version validation.
    ///
    /// Supports version 1 (uncompressed), version 2 (zstd), and
    /// headerless legacy format.
    fn load(path: &PathBuf) -> std::io::Result<Self> {
        use std::io::Read;

        // Sanity check: reject files over 100MB to prevent OOM from corrupted recordings
        const MAX_RECORDING_SIZE: u64 = 100 * 1024 * 1024;
        let file_size = std::fs::metadata(path)?.len();
        if file_size > MAX_RECORDING_SIZE {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "Recording file too large ({:.1} MB, max {} MB): {}",
                    file_size as f64 / 1024.0 / 1024.0,
                    MAX_RECORDING_SIZE / 1024 / 1024,
                    path.display()
                ),
            ));
        }

        let file = File::open(path)?;
        let mut reader = BufReader::new(file);

        // Try to read the header
        let mut magic_buf = [0u8; 6];
        match reader.read_exact(&mut magic_buf) {
            Ok(()) if &magic_buf == RECORDING_MAGIC => {
                // Versioned file — read version
                let mut version_buf = [0u8; 4];
                reader.read_exact(&mut version_buf)?;
                let version = u32::from_le_bytes(version_buf);

                match version {
                    RECORDING_FORMAT_VERSION => {
                        // Version 1: uncompressed bincode
                        // File size already validated above (MAX_RECORDING_SIZE)
                        let mut data = Vec::new();
                        reader.read_to_end(&mut data)?;
                        bincode::deserialize(&data)
                            .map_err(|e| std::io::Error::other(e.to_string()))
                    }
                    #[cfg(feature = "recording-compression")]
                    RECORDING_FORMAT_VERSION_ZSTD => {
                        // Version 2: zstd-compressed bincode
                        let mut compressed = Vec::new();
                        reader.read_to_end(&mut compressed)?;
                        let decompressed =
                            zstd::decode_all(compressed.as_slice()).map_err(|e| {
                                std::io::Error::other(format!("zstd decompress: {}", e))
                            })?;
                        bincode::deserialize(&decompressed)
                            .map_err(|e| std::io::Error::other(e.to_string()))
                    }
                    _ => Err(std::io::Error::other(format!(
                        "Unsupported recording format version {} (supported: 1{}). File: {}",
                        version,
                        if cfg!(feature = "recording-compression") {
                            ", 2"
                        } else {
                            ""
                        },
                        path.display()
                    ))),
                }
            }
            _ => {
                // No valid magic — try legacy headerless bincode from start
                let file = File::open(path)?;
                let reader = BufReader::new(file);
                bincode::deserialize_from(reader).map_err(|e| {
                    std::io::Error::other(format!(
                        "Failed to load recording (no version header, legacy format): {}. File: {}",
                        e,
                        path.display()
                    ))
                })
            }
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
    /// Source host identifier for network-replicated data.
    /// None = local, Some("host_id") = data from remote machine via horus_net.
    /// Added in schema v2. Missing in older recordings (defaults to None).
    #[serde(default)]
    pub source_host: Option<String>,
    /// Whether this snapshot contains data from a remote machine.
    #[serde(default)]
    pub is_remote: bool,
}

impl NodeTickSnapshot {
    pub fn new(tick: u64) -> Self {
        Self::with_capacity(tick, 0)
    }

    /// Create a snapshot with pre-allocated HashMap capacity.
    ///
    /// Avoids per-tick heap allocations when capacity is known from
    /// prior ticks, reducing jitter in RT recording paths.
    pub fn with_capacity(tick: u64, capacity: usize) -> Self {
        Self {
            tick,
            timestamp_us: SystemTime::now()
                .duration_since(SystemTime::UNIX_EPOCH)
                .unwrap_or_default()
                .as_micros() as u64,
            inputs: HashMap::with_capacity(capacity),
            outputs: HashMap::with_capacity(capacity),
            state: None,
            duration_ns: 0,
            source_host: None,
            is_remote: false,
        }
    }

    /// Create a snapshot with pre-allocated capacity and a monotonic timestamp.
    ///
    /// Uses a pre-computed timestamp derived from `Instant` to avoid
    /// `SystemTime::now()` calls in the hot path and prevent NTP clock jumps.
    pub fn with_capacity_and_timestamp(tick: u64, capacity: usize, timestamp_us: u64) -> Self {
        Self {
            tick,
            timestamp_us,
            inputs: HashMap::with_capacity(capacity),
            outputs: HashMap::with_capacity(capacity),
            state: None,
            duration_ns: 0,
            source_host: None,
            is_remote: false,
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

/// Current schema version for new recordings.
pub const RECORDING_SCHEMA_VERSION: u32 = 2;

/// Recording of a single node
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct NodeRecording {
    /// Schema version for forward/backward compatibility.
    /// v0 = legacy (no schema field), v1 = original, v2 = added source_host/is_remote.
    /// Missing in old recordings → defaults to 0 (legacy).
    #[serde(default)]
    pub schema_version: u32,
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
    /// Message type names per topic (topic -> type name).
    /// Populated once during recording so tools can decode data blobs.
    #[serde(default)]
    pub topic_types: HashMap<String, String>,
}

impl NodeRecording {
    pub fn new(node_name: &str, node_id: &str, session_name: &str) -> Self {
        Self {
            schema_version: RECORDING_SCHEMA_VERSION,
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
            topic_types: HashMap::new(),
        }
    }

    /// Register a topic's message type name.
    pub fn set_topic_type(&mut self, topic: &str, type_name: &str) {
        self.topic_types
            .entry(topic.to_string())
            .or_insert_with(|| type_name.to_string());
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
    pub fn snapshot(&self, tick: u64) -> Option<&NodeTickSnapshot> {
        self.snapshots.iter().find(|s| s.tick == tick)
    }

    /// Get snapshots in a tick range
    pub fn snapshots_range(&self, start_tick: u64, end_tick: u64) -> Vec<&NodeTickSnapshot> {
        self.snapshots
            .iter()
            .filter(|s| s.tick >= start_tick && s.tick <= end_tick)
            .collect()
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
}

impl Recording for NodeRecording {
    fn ended_at_mut(&mut self) -> &mut Option<u64> {
        &mut self.ended_at
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
}

impl Recording for SchedulerRecording {
    fn ended_at_mut(&mut self) -> &mut Option<u64> {
        &mut self.ended_at
    }
}

/// Active recorder for a node
pub struct NodeRecorder {
    recording: NodeRecording,
    config: RecordingConfig,
    current_snapshot: Option<NodeTickSnapshot>,
    enabled: bool,
    /// Estimated accumulated recording size in bytes.
    estimated_size: u64,
    /// Pre-allocated capacity hint for inputs/outputs HashMaps.
    /// Set once after first tick to avoid per-tick allocations.
    io_capacity: usize,
    /// Monotonic base instant (set at recorder creation).
    /// Used to compute timestamps that are immune to NTP clock adjustments.
    mono_base: Instant,
    /// Wall-clock epoch at `mono_base` (for converting back to absolute time).
    epoch_base_us: u64,
}

impl NodeRecorder {
    pub(crate) fn new(node_name: &str, node_id: &str, config: RecordingConfig) -> Self {
        let now_wall = SystemTime::now()
            .duration_since(SystemTime::UNIX_EPOCH)
            .unwrap_or_default()
            .as_micros() as u64;
        Self {
            recording: NodeRecording::new(node_name, node_id, &config.session_name),
            config,
            current_snapshot: None,
            enabled: true,
            estimated_size: 0,
            io_capacity: 0,
            mono_base: Instant::now(),
            epoch_base_us: now_wall,
        }
    }

    /// Start recording a new tick.
    ///
    /// Uses pre-allocated HashMap capacity after the first tick to reduce
    /// allocation pressure in the recording hot path.
    pub(crate) fn begin_tick(&mut self, tick: u64) {
        if !self.enabled {
            return;
        }

        // Check recording interval (interval is guaranteed >= 1 by validation)
        let interval = self.config.interval.max(1);
        if !tick.is_multiple_of(interval) {
            self.current_snapshot = None;
            return;
        }

        let elapsed_us = self.mono_base.elapsed().as_micros() as u64;
        let timestamp_us = self.epoch_base_us + elapsed_us;
        self.current_snapshot = Some(NodeTickSnapshot::with_capacity_and_timestamp(
            tick,
            self.io_capacity,
            timestamp_us,
        ));
    }

    /// Finish recording the current tick
    pub(crate) fn end_tick(&mut self, duration_ns: u64) {
        if let Some(mut snapshot) = self.current_snapshot.take() {
            if self.config.record_timing {
                snapshot.duration_ns = duration_ns;
            }

            // Learn capacity from first populated snapshot to pre-allocate future ones
            if self.io_capacity == 0 {
                let cap = snapshot.inputs.len().max(snapshot.outputs.len());
                if cap > 0 {
                    self.io_capacity = cap;
                }
            }

            // Estimate snapshot size: fixed overhead + data bytes
            let snapshot_size: u64 = 64 // tick, duration, timestamps, map overhead
                + snapshot.inputs.values().map(|v| v.len() as u64).sum::<u64>()
                + snapshot.outputs.values().map(|v| v.len() as u64).sum::<u64>();
            self.estimated_size += snapshot_size;

            // Enforce max_size_bytes (0 = unlimited)
            if self.config.max_size_bytes > 0 && self.estimated_size > self.config.max_size_bytes {
                self.enabled = false;
                return;
            }

            self.recording.add_snapshot(snapshot);

            // Ring-buffer mode: evict oldest snapshots when limit exceeded
            if self.config.max_snapshots > 0
                && self.recording.snapshots.len() > self.config.max_snapshots
            {
                let excess = self.recording.snapshots.len() - self.config.max_snapshots;
                self.recording.snapshots.drain(..excess);
                if let Some(first) = self.recording.snapshots.first() {
                    self.recording.first_tick = first.tick;
                }
            }
        }
    }

    /// Finish and save the recording
    pub(crate) fn finish(&mut self) -> std::io::Result<PathBuf> {
        self.recording.finish();
        self.enabled = false;

        let path = self
            .config
            .node_path(&self.recording.node_name, &self.recording.node_id);

        if self.recording.snapshots.is_empty() {
            return Err(std::io::Error::other(format!(
                "Recording for '{}' has no snapshots — nothing to save",
                self.recording.node_name
            )));
        }

        self.recording
            .save_with_compression(&path, self.config.compress)?;

        Ok(path)
    }
}

impl NodeRecorder {
    /// Get a reference to the underlying recording.
    #[cfg(test)]
    pub(crate) fn recording(&self) -> &NodeRecording {
        &self.recording
    }

    /// Whether a snapshot is being captured for the current tick.
    ///
    /// Returns `false` when recording is disabled or the current tick was
    /// skipped by the interval filter.  The scheduler checks this to avoid
    /// the overhead of reading topic shared-memory on non-recorded ticks.
    pub(crate) fn is_active_tick(&self) -> bool {
        self.current_snapshot.is_some()
    }

    /// Record an input received during the current tick.
    ///
    /// Called by the scheduler **before** `node.tick()` for each subscriber
    /// topic.  The data is the raw bytes read from the topic's shared-memory
    /// ring buffer via `read_latest_slot_bytes`.
    pub(crate) fn record_input(&mut self, topic: &str, data: Vec<u8>) {
        if self.config.record_inputs {
            if let Some(ref mut snapshot) = self.current_snapshot {
                snapshot.inputs.insert(topic.to_string(), data);
            }
        }
    }

    /// Record an output produced during the current tick.
    ///
    /// Called by the scheduler **after** `node.tick()` for each publisher
    /// topic.  The data is the raw bytes read from the topic's shared-memory
    /// ring buffer via `read_latest_slot_bytes`.
    pub(crate) fn record_output(&mut self, topic: &str, data: Vec<u8>) {
        if self.config.record_outputs {
            if let Some(ref mut snapshot) = self.current_snapshot {
                snapshot.outputs.insert(topic.to_string(), data);
            }
        }
    }
}

/// Replayer for a node recording
pub struct NodeReplayer {
    recording: NodeRecording,
    current_index: usize,
    current_tick: u64,
}

impl NodeReplayer {
    /// Load a recording from file.
    ///
    /// Returns an error if the recording contains no snapshots.
    pub fn load(path: &PathBuf) -> std::io::Result<Self> {
        let recording = NodeRecording::load(path)?;
        if recording.snapshots.is_empty() {
            return Err(std::io::Error::new(
                std::io::ErrorKind::InvalidData,
                format!(
                    "Recording '{}' from {} has no snapshots",
                    recording.node_name,
                    path.display()
                ),
            ));
        }
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
    pub fn outputs(&self) -> Option<&HashMap<String, Vec<u8>>> {
        self.current_snapshot().map(|s| &s.outputs)
    }

    /// Get a specific output for the current tick
    pub fn output(&self, topic: &str) -> Option<&Vec<u8>> {
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

    /// Get current snapshot index within the recording.
    pub fn current_index(&self) -> usize {
        self.current_index
    }

    /// Get total ticks in recording
    pub fn total_ticks(&self) -> usize {
        self.recording.snapshots.len()
    }
}

/// Manager for session discovery.
///
/// Searches multiple directories for recording sessions:
/// 1. The canonical platform data directory (`data_dir()/recordings`).
/// 2. A CWD-local `.horus/recordings/` directory (project-local recordings).
///
/// Results are deduplicated so a session that exists in both locations
/// appears only once (the platform directory takes precedence for path
/// resolution in [`session_recordings`] and [`delete_session`]).
pub struct RecordingManager {
    /// Primary search directory (platform data dir).
    base_dir: PathBuf,
    /// Additional search directories (e.g. CWD-local `.horus/recordings`).
    extra_dirs: Vec<PathBuf>,
}

impl RecordingManager {
    pub fn new() -> Self {
        let base_dir = default_recordings_dir();

        // Also check CWD-local .horus/recordings (project-local recordings).
        let local = PathBuf::from(RECORDINGS_DIR);
        let extra_dirs = if local.is_dir() {
            // Canonicalize to avoid duplicates when CWD is inside the data dir.
            let base_canon = fs::canonicalize(&base_dir).unwrap_or_else(|_| base_dir.clone());
            let local_canon = fs::canonicalize(&local).unwrap_or_else(|_| local.clone());
            if base_canon != local_canon {
                vec![local]
            } else {
                vec![]
            }
        } else {
            vec![]
        };

        Self {
            base_dir,
            extra_dirs,
        }
    }

    pub fn with_base_dir(base_dir: PathBuf) -> Self {
        Self {
            base_dir,
            extra_dirs: vec![],
        }
    }

    /// Collect all search directories (base + extras).
    fn all_dirs(&self) -> Vec<&PathBuf> {
        let mut dirs = vec![&self.base_dir];
        dirs.extend(&self.extra_dirs);
        dirs
    }

    /// List all recording sessions across all search directories.
    pub fn list_sessions(&self) -> std::io::Result<Vec<String>> {
        let mut sessions = Vec::new();
        let mut seen = std::collections::HashSet::new();

        for dir in self.all_dirs() {
            if dir.exists() {
                for entry in fs::read_dir(dir)? {
                    let entry = entry?;
                    if entry.file_type()?.is_dir() {
                        if let Some(name) = entry.file_name().to_str() {
                            if seen.insert(name.to_string()) {
                                sessions.push(name.to_string());
                            }
                        }
                    }
                }
            }
        }

        Ok(sessions)
    }

    /// Find the actual session directory across all search directories.
    ///
    /// Returns the first existing match (base_dir checked first).
    fn find_session_dir(&self, session: &str) -> PathBuf {
        let safe_name = sanitize_path_component(session);
        for dir in self.all_dirs() {
            let candidate = dir.join(&safe_name);
            if candidate.exists() {
                return candidate;
            }
        }
        // Fallback to base_dir (may not exist yet).
        self.base_dir.join(safe_name)
    }

    /// Get all recordings in a session.
    ///
    /// Session name is sanitized to prevent directory traversal.
    /// Searches all known directories for the session.
    pub fn session_recordings(&self, session: &str) -> std::io::Result<Vec<PathBuf>> {
        let session_dir = self.find_session_dir(session);
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

    /// Delete a session and all its recordings.
    ///
    /// Session name is sanitized to prevent directory traversal.
    /// Deletes from whichever directory contains the session.
    pub fn delete_session(&self, session: &str) -> std::io::Result<()> {
        let session_dir = self.find_session_dir(session);
        if session_dir.exists() {
            fs::remove_dir_all(session_dir)?;
        }
        Ok(())
    }

    /// Get total size of recordings across all search directories.
    pub fn total_size(&self) -> std::io::Result<u64> {
        let mut total = 0;

        for session in self.list_sessions()? {
            for path in self.session_recordings(&session)? {
                if let Ok(metadata) = fs::metadata(&path) {
                    total += metadata.len();
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

/// Compare two recordings for differences.
///
/// Uses O(n + m) HashMap-based comparison instead of iterating over the
/// potentially huge tick range (which was O(range × snapshots) when
/// recordings are sparse).
pub fn diff_recordings(
    recording1: &NodeRecording,
    recording2: &NodeRecording,
) -> Vec<RecordingDiff> {
    let mut diffs = Vec::new();

    // Build tick → snapshot index maps for O(1) lookup
    let map1: HashMap<u64, &NodeTickSnapshot> =
        recording1.snapshots.iter().map(|s| (s.tick, s)).collect();
    let map2: HashMap<u64, &NodeTickSnapshot> =
        recording2.snapshots.iter().map(|s| (s.tick, s)).collect();

    // Collect the union of all ticks from both recordings within the
    // overlapping range, then iterate only those ticks.
    let start = recording1.first_tick.max(recording2.first_tick);
    let end = recording1.last_tick.min(recording2.last_tick);

    // Gather all unique ticks in the overlap range from both recordings
    let mut all_ticks: Vec<u64> = map1
        .keys()
        .chain(map2.keys())
        .copied()
        .filter(|&t| t >= start && t <= end)
        .collect();
    all_ticks.sort_unstable();
    all_ticks.dedup();

    for tick in all_ticks {
        let snap1 = map1.get(&tick);
        let snap2 = map2.get(&tick);

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
    tick_count: u64,
}

impl ReplayNode {
    /// Create a new replay node.
    pub fn new(node_name: String) -> Self {
        Self {
            node_name,
            tick_count: 0,
        }
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
                index: self.replayer.current_index(),
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
                index: self.replayer.current_index(),
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
            index: self.replayer.current_index(),
        });

        // Evaluate watches
        self.evaluate_watches();

        // Check breakpoints (single pass — no double evaluation)
        if check_breakpoints {
            if let Some(snapshot) = self.replayer.current_snapshot() {
                let mut first_hit_id = None;
                for bp in &mut self.breakpoints {
                    if bp.should_trigger(snapshot, current_tick) {
                        bp.hit_count += 1;
                        if first_hit_id.is_none() {
                            first_hit_id = Some(bp.id);
                        }
                    }
                }

                if let Some(bp_id) = first_hit_id {
                    self.state = DebuggerState::Paused;
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

        let snap = recording.snapshot(1).unwrap();
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
        recorder.end_tick(1000);

        recorder.begin_tick(1);
        recorder.end_tick(2000);

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
        assert_eq!(replayer.output("motor").unwrap(), &vec![1, 2, 3]);

        replayer.advance();
        assert_eq!(replayer.current_tick(), 1);
        assert_eq!(replayer.output("motor").unwrap(), &vec![4, 5, 6]);

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
            other => unreachable!("Expected OutputDifference, got {:?}", other),
        }
    }

    #[test]
    fn test_recording_interval() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "test".to_string(),
            base_dir: dir.path().to_path_buf(),
            interval: 2, // Record every 2 ticks
            ..Default::default()
        };

        let mut recorder = NodeRecorder::new("test_node", "abc123", config);

        recorder.begin_tick(0);
        recorder.end_tick(100);

        recorder.begin_tick(1); // Should be skipped (not multiple of 2)
        recorder.end_tick(100);

        recorder.begin_tick(2);
        recorder.end_tick(100);

        // Finish and load to verify snapshot count
        let path = recorder.finish().unwrap();
        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.snapshot_count(), 2); // Only ticks 0 and 2
    }

    #[test]
    fn test_record_inputs_disabled() {
        let mut config = RecordingConfig::default();
        config.record_inputs = false;

        let mut recorder = NodeRecorder::new("test_node", "test-id-1", config);

        recorder.begin_tick(0);
        recorder.record_input("sensor.data", vec![1, 2, 3]);
        recorder.record_output("motor.cmd", vec![4, 5, 6]);
        recorder.end_tick(1000);

        let recording = recorder.recording();
        let snap = &recording.snapshots[0];
        assert!(
            snap.inputs.is_empty(),
            "inputs should be empty when record_inputs=false"
        );
        assert!(!snap.outputs.is_empty(), "outputs should still be recorded");
    }

    #[test]
    fn test_record_outputs_disabled() {
        let mut config = RecordingConfig::default();
        config.record_outputs = false;

        let mut recorder = NodeRecorder::new("test_node", "test-id-2", config);

        recorder.begin_tick(0);
        recorder.record_input("sensor.data", vec![1, 2, 3]);
        recorder.record_output("motor.cmd", vec![4, 5, 6]);
        recorder.end_tick(1000);

        let recording = recorder.recording();
        let snap = &recording.snapshots[0];
        assert!(!snap.inputs.is_empty(), "inputs should still be recorded");
        assert!(
            snap.outputs.is_empty(),
            "outputs should be empty when record_outputs=false"
        );
    }

    #[test]
    fn test_record_timing_disabled() {
        let mut config = RecordingConfig::default();
        config.record_timing = false;

        let mut recorder = NodeRecorder::new("test_node", "test-id-3", config);

        recorder.begin_tick(0);
        recorder.end_tick(99999);

        let recording = recorder.recording();
        let snap = &recording.snapshots[0];
        assert_eq!(
            snap.duration_ns, 0,
            "duration_ns should be 0 when record_timing=false"
        );
    }

    #[test]
    fn test_all_recording_enabled() {
        let config = RecordingConfig::default();
        assert!(config.record_inputs);
        assert!(config.record_outputs);
        assert!(config.record_timing);

        let mut recorder = NodeRecorder::new("test_node", "test-id-4", config);

        recorder.begin_tick(0);
        recorder.record_input("sensor.data", vec![1, 2, 3]);
        recorder.record_output("motor.cmd", vec![4, 5, 6]);
        recorder.end_tick(50000);

        let recording = recorder.recording();
        let snap = &recording.snapshots[0];
        assert!(
            !snap.inputs.is_empty(),
            "inputs should be recorded by default"
        );
        assert!(
            !snap.outputs.is_empty(),
            "outputs should be recorded by default"
        );
        assert_eq!(
            snap.duration_ns, 50000,
            "duration should be recorded by default"
        );
    }

    #[test]
    fn test_from_yaml_transfers_flags() {
        let mut yaml = super::super::config::RecordingConfigYaml::default();
        yaml.record_inputs = false;
        yaml.record_outputs = false;
        yaml.record_timing = false;

        let config: RecordingConfig = yaml.into();
        assert!(!config.record_inputs);
        assert!(!config.record_outputs);
        assert!(!config.record_timing);
    }

    // ---- Phase 6: ReplayDebugger tests ----

    fn make_test_recording(ticks: u64) -> NodeRecording {
        let mut rec = NodeRecording::new("test", "id1", "session");
        for t in 0..ticks {
            rec.add_snapshot(
                NodeTickSnapshot::new(t)
                    .with_input("sensor", vec![t as u8; 4])
                    .with_output("motor", vec![(t * 2) as u8; 8]),
            );
        }
        rec
    }

    #[test]
    fn test_debugger_step_forward() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        assert_eq!(dbg.state(), DebuggerState::Paused);
        assert_eq!(dbg.current_tick(), 0);

        assert!(dbg.step_forward());
        assert_eq!(dbg.current_tick(), 1);
        assert_eq!(dbg.state(), DebuggerState::Paused);

        assert!(dbg.step_forward());
        assert_eq!(dbg.current_tick(), 2);
    }

    #[test]
    fn test_debugger_step_backward() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        dbg.step_forward(); // tick 1
        dbg.step_forward(); // tick 2
        assert_eq!(dbg.current_tick(), 2);

        assert!(dbg.step_backward());
        assert_eq!(dbg.current_tick(), 1);
        assert_eq!(dbg.state(), DebuggerState::Paused);
    }

    #[test]
    fn test_debugger_step_backward_at_start() {
        let rec = make_test_recording(3);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        // At tick 0, stepping backward should fail
        assert!(!dbg.step_backward());
        assert_eq!(dbg.current_tick(), 0);
    }

    #[test]
    fn test_debugger_seek() {
        let rec = make_test_recording(10);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        assert!(dbg.seek(5));
        assert_eq!(dbg.current_tick(), 5);

        assert!(dbg.seek(0));
        assert_eq!(dbg.current_tick(), 0);
    }

    #[test]
    fn test_debugger_reset() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        dbg.step_forward();
        dbg.step_forward();
        dbg.reset();
        assert_eq!(dbg.current_tick(), 0);
        assert_eq!(dbg.state(), DebuggerState::Paused);
    }

    #[test]
    fn test_debugger_breakpoint_at_tick() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        let bp_id = dbg.add_breakpoint(BreakpointCondition::AtTick(2));
        assert_eq!(bp_id, 1);

        // Run until breakpoint hit
        dbg.continue_execution();
        assert_eq!(dbg.state(), DebuggerState::Paused);
        assert_eq!(dbg.current_tick(), 2);

        // Check breakpoint hit count
        let bp = dbg.breakpoints().iter().find(|b| b.id == bp_id).unwrap();
        assert_eq!(bp.hit_count, 1);
    }

    #[test]
    fn test_debugger_breakpoint_topic_has_data() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        dbg.add_breakpoint(BreakpointCondition::TopicHasData("sensor".to_string()));

        // Should hit on tick 1 (first advance)
        dbg.continue_execution();
        assert_eq!(dbg.state(), DebuggerState::Paused);
    }

    #[test]
    fn test_debugger_disable_breakpoint() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        let bp_id = dbg.add_breakpoint(BreakpointCondition::AtTick(2));
        dbg.set_breakpoint_enabled(bp_id, false);

        // Should run past tick 2 since breakpoint is disabled
        dbg.continue_execution();
        assert_eq!(dbg.state(), DebuggerState::Stopped);
    }

    #[test]
    fn test_debugger_remove_breakpoint() {
        let rec = make_test_recording(5);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        let bp_id = dbg.add_breakpoint(BreakpointCondition::AtTick(2));
        assert!(dbg.remove_breakpoint(bp_id));
        assert!(dbg.breakpoints().iter().find(|b| b.id == bp_id).is_none());
    }

    #[test]
    fn test_debugger_events_emitted() {
        let rec = make_test_recording(3);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        dbg.step_forward();
        let events = dbg.events();
        assert!(!events.is_empty());
    }

    // ---- WatchExpression tests ----

    #[test]
    fn test_watch_output_full() {
        let snapshot = NodeTickSnapshot::new(0).with_output("motor", vec![1, 2, 3, 4]);
        let watch = WatchExpression::output("w1", "Motor Output", "motor");
        let val = watch.evaluate(&snapshot).unwrap();
        assert_eq!(val.raw_bytes, vec![1, 2, 3, 4]);
        assert_eq!(val.tick, 0);
        assert_eq!(val.expression_id, "w1");
    }

    #[test]
    fn test_watch_input_full() {
        let snapshot = NodeTickSnapshot::new(5).with_input("sensor", vec![10, 20]);
        let watch = WatchExpression::input("w2", "Sensor Input", "sensor");
        let val = watch.evaluate(&snapshot).unwrap();
        assert_eq!(val.raw_bytes, vec![10, 20]);
    }

    #[test]
    fn test_watch_missing_topic() {
        let snapshot = NodeTickSnapshot::new(0);
        let watch = WatchExpression::output("w1", "Missing", "nonexistent");
        assert!(watch.evaluate(&snapshot).is_none());
    }

    #[test]
    fn test_watch_with_byte_range() {
        let snapshot = NodeTickSnapshot::new(0).with_output("data", vec![0, 1, 2, 3, 4, 5, 6, 7]);
        let watch = WatchExpression::output("w1", "Slice", "data").with_range(2, 3);
        let val = watch.evaluate(&snapshot).unwrap();
        assert_eq!(val.raw_bytes, vec![2, 3, 4]);
    }

    #[test]
    fn test_watch_byte_range_out_of_bounds() {
        let snapshot = NodeTickSnapshot::new(0).with_output("data", vec![1, 2]);
        let watch = WatchExpression::output("w1", "OOB", "data").with_range(1, 5);
        assert!(watch.evaluate(&snapshot).is_none());
    }

    #[test]
    fn test_watch_offset_only() {
        let snapshot = NodeTickSnapshot::new(0).with_output("data", vec![10, 20, 30, 40]);
        let watch = WatchExpression::output("w1", "Tail", "data");
        let mut w = watch;
        w.byte_offset = Some(2);
        w.byte_length = None;
        let val = w.evaluate(&snapshot).unwrap();
        assert_eq!(val.raw_bytes, vec![30, 40]);
    }

    // ---- format_bytes_as_value tests ----

    #[test]
    fn test_format_bytes_u8() {
        let s = format_bytes_as_value(&[42]);
        assert!(s.contains("u8: 42"));
    }

    #[test]
    fn test_format_bytes_f64() {
        let val = 2.75f64;
        let s = format_bytes_as_value(&val.to_le_bytes());
        assert!(s.contains("f64:"));
        assert!(s.contains("2.75"));
    }

    #[test]
    fn test_format_bytes_string() {
        let s = format_bytes_as_value(b"hello");
        assert!(s.contains("str: \"hello\""));
    }

    #[test]
    fn test_format_bytes_large() {
        let data = vec![0xFFu8; 64];
        let s = format_bytes_as_value(&data);
        assert!(s.contains("bytes[64]"));
        assert!(s.contains("..."));
    }

    // ---- DebugSessionState tests ----

    #[test]
    fn test_debug_session_state_save_load() {
        let dir = tempdir().unwrap();
        let state_path = dir.path().join("session.json");

        let mut state = DebugSessionState::new(PathBuf::from("/tmp/test.horus"), "test_session");
        state.current_tick = 42;
        state.breakpoints.push(Breakpoint {
            id: 1,
            condition: BreakpointCondition::AtTick(10),
            enabled: true,
            hit_count: 3,
            name: None,
        });
        state
            .watches
            .push(WatchExpression::output("w1", "Motor", "motor"));

        state.save(&state_path).unwrap();

        let loaded = DebugSessionState::load(&state_path).unwrap();
        assert_eq!(loaded.current_tick, 42);
        assert_eq!(loaded.session_name, "test_session");
        assert_eq!(loaded.breakpoints.len(), 1);
        assert_eq!(loaded.breakpoints[0].hit_count, 3);
        assert_eq!(loaded.watches.len(), 1);
        assert_eq!(loaded.watches[0].topic, "motor");
    }

    // ---- diff_recordings edge case tests ----

    #[test]
    fn test_diff_identical_recordings() {
        let mut r1 = NodeRecording::new("n", "1", "s");
        let mut r2 = NodeRecording::new("n", "2", "s");
        for t in 0..5 {
            r1.add_snapshot(NodeTickSnapshot::new(t).with_output("out", vec![t as u8]));
            r2.add_snapshot(NodeTickSnapshot::new(t).with_output("out", vec![t as u8]));
        }
        let diffs = diff_recordings(&r1, &r2);
        assert!(
            diffs.is_empty(),
            "identical recordings should produce no diffs"
        );
    }

    #[test]
    fn test_diff_sparse_non_overlapping() {
        let mut r1 = NodeRecording::new("n", "1", "s");
        let mut r2 = NodeRecording::new("n", "2", "s");
        // r1 has ticks 0,2,4; r2 has ticks 1,3,5
        r1.add_snapshot(NodeTickSnapshot::new(0).with_output("out", vec![1]));
        r1.add_snapshot(NodeTickSnapshot::new(2).with_output("out", vec![2]));
        r1.add_snapshot(NodeTickSnapshot::new(4).with_output("out", vec![3]));
        r2.add_snapshot(NodeTickSnapshot::new(1).with_output("out", vec![4]));
        r2.add_snapshot(NodeTickSnapshot::new(3).with_output("out", vec![5]));
        r2.add_snapshot(NodeTickSnapshot::new(5).with_output("out", vec![6]));

        let diffs = diff_recordings(&r1, &r2);
        // Overlapping range is [1,4], so ticks 1,2,3,4 are relevant
        // Tick 1: only in r2 -> MissingTick in recording1
        // Tick 2: only in r1 -> MissingTick in recording2
        // Tick 3: only in r2 -> MissingTick in recording1
        // Tick 4: only in r1 -> MissingTick in recording2
        assert!(!diffs.is_empty());
    }

    #[test]
    fn test_diff_empty_recordings() {
        let r1 = NodeRecording::new("n", "1", "s");
        let r2 = NodeRecording::new("n", "2", "s");
        let diffs = diff_recordings(&r1, &r2);
        assert!(diffs.is_empty());
    }

    #[test]
    fn test_diff_single_tick_difference() {
        let mut r1 = NodeRecording::new("n", "1", "s");
        let mut r2 = NodeRecording::new("n", "2", "s");
        r1.add_snapshot(NodeTickSnapshot::new(5).with_output("cmd", vec![0xFF]));
        r2.add_snapshot(NodeTickSnapshot::new(5).with_output("cmd", vec![0x00]));

        let diffs = diff_recordings(&r1, &r2);
        assert_eq!(diffs.len(), 1);
    }

    // ---- Replay output verification tests ----

    #[test]
    fn test_replayer_outputs_sequence() {
        let mut rec = NodeRecording::new("motor", "m1", "session");
        for t in 0..5 {
            rec.add_snapshot(NodeTickSnapshot::new(t).with_output("cmd", vec![t as u8]));
        }
        let mut replayer = NodeReplayer::from_recording(rec);

        for t in 0..5u64 {
            let snap = replayer.current_snapshot().unwrap();
            assert_eq!(snap.outputs.get("cmd").unwrap(), &vec![t as u8]);
            if t < 4 {
                assert!(replayer.advance(), "should advance at tick {}", t);
            } else {
                assert!(!replayer.advance(), "should not advance past last tick");
            }
        }
    }

    #[test]
    fn test_replayer_seek_and_read() {
        let mut rec = NodeRecording::new("n", "id", "s");
        for t in 0..10 {
            rec.add_snapshot(NodeTickSnapshot::new(t).with_output("out", vec![t as u8]));
        }
        let mut replayer = NodeReplayer::from_recording(rec);

        replayer.seek(7);
        assert_eq!(replayer.current_tick(), 7);
        assert_eq!(
            replayer
                .current_snapshot()
                .unwrap()
                .outputs
                .get("out")
                .unwrap(),
            &vec![7u8]
        );
    }

    #[test]
    fn test_replayer_reset_after_end() {
        let mut rec = NodeRecording::new("n", "id", "s");
        rec.add_snapshot(NodeTickSnapshot::new(0).with_output("o", vec![1]));
        rec.add_snapshot(NodeTickSnapshot::new(1).with_output("o", vec![2]));

        let mut replayer = NodeReplayer::from_recording(rec);
        assert!(replayer.advance()); // index 0 -> 1
        assert!(!replayer.advance()); // at last, returns false

        // At last snapshot, not "finished" (still valid current_snapshot)
        assert_eq!(replayer.current_tick(), 1);

        replayer.reset();
        assert_eq!(replayer.current_tick(), 0);
        assert_eq!(replayer.current_index(), 0);
    }

    // ---- Recording size limit and ring buffer tests ----

    #[test]
    fn test_max_size_stops_recording() {
        let config = RecordingConfig {
            session_name: "test".into(),
            base_dir: PathBuf::from("/tmp"),
            max_size_bytes: 200, // Very small limit
            ..Default::default()
        };
        let mut recorder = NodeRecorder::new("n", "id", config);

        for t in 0..100 {
            recorder.begin_tick(t);
            recorder.record_output("big", vec![0u8; 50]);
            recorder.end_tick(100);
        }

        // Should have stopped before recording all 100 ticks
        let snap_count = recorder.recording().snapshot_count();
        assert!(
            snap_count < 100,
            "max_size should cap recording, got {} snapshots",
            snap_count
        );
    }

    #[test]
    fn test_ring_buffer_mode() {
        let config = RecordingConfig {
            session_name: "test".into(),
            base_dir: PathBuf::from("/tmp"),
            max_snapshots: 5,
            ..Default::default()
        };
        let mut recorder = NodeRecorder::new("n", "id", config);

        for t in 0..20 {
            recorder.begin_tick(t);
            recorder.record_output("out", vec![t as u8]);
            recorder.end_tick(100);
        }

        let rec = recorder.recording();
        assert_eq!(
            rec.snapshot_count(),
            5,
            "ring buffer should keep only 5 snapshots"
        );
        assert_eq!(
            rec.first_tick, 15,
            "first_tick should be updated after eviction"
        );
        assert_eq!(rec.last_tick, 19);
    }

    // ---- Path sanitization tests ----

    #[test]
    fn test_sanitize_path_traversal() {
        let result = sanitize_path_component("../../etc/passwd");
        assert!(!result.contains(".."));
        assert!(!result.contains('/'));
        assert_eq!(result, "passwd");
    }

    #[test]
    fn test_sanitize_empty() {
        assert_eq!(sanitize_path_component(""), "unnamed");
    }

    #[test]
    fn test_sanitize_normal() {
        assert_eq!(sanitize_path_component("motor_node"), "motor_node");
    }

    #[test]
    fn test_sanitize_absolute() {
        let result = sanitize_path_component("/root/secret");
        assert!(!result.starts_with('/'));
    }

    // ---- Session name collision test ----

    #[test]
    fn test_session_name_has_subsecond_precision() {
        let config = RecordingConfig::default();
        // Should contain milliseconds and process ID
        assert!(config.session_name.contains('_'));
        assert!(
            config.session_name.contains("_p"),
            "session name should contain _p<pid>"
        );
    }

    // ---- Empty recording handling tests ----

    #[test]
    fn test_empty_recording_finish_fails() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "test".into(),
            base_dir: dir.path().to_path_buf(),
            ..Default::default()
        };
        let mut recorder = NodeRecorder::new("empty_node", "id1", config);
        // No ticks recorded
        let result = recorder.finish();
        assert!(result.is_err(), "finish() on empty recording should error");
    }

    #[test]
    fn test_empty_recording_load_fails() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("empty.horus");

        // Create and save an empty recording directly
        let rec = NodeRecording::new("empty", "id", "session");
        rec.save(&path).unwrap();

        // Loading as replayer should fail
        let result = NodeReplayer::load(&path);
        assert!(result.is_err());
    }

    // ---- Versioned save/load tests ----

    #[test]
    fn test_versioned_save_load_roundtrip() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("versioned.horus");

        let mut rec = NodeRecording::new("v_node", "v_id", "v_session");
        rec.add_snapshot(NodeTickSnapshot::new(0).with_output("out", vec![42]));
        rec.finish();
        rec.save(&path).unwrap();

        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.node_name, "v_node");
        assert_eq!(loaded.snapshot_count(), 1);
        assert_eq!(
            loaded.snapshot(0).unwrap().outputs.get("out").unwrap(),
            &vec![42u8]
        );
    }

    #[test]
    fn test_versioned_file_has_magic() {
        use std::io::Read;

        let dir = tempdir().unwrap();
        let path = dir.path().join("magic.horus");

        let mut rec = NodeRecording::new("m", "id", "s");
        rec.add_snapshot(NodeTickSnapshot::new(0));
        rec.save(&path).unwrap();

        let mut file = File::open(&path).unwrap();
        let mut magic = [0u8; 6];
        file.read_exact(&mut magic).unwrap();
        assert_eq!(&magic, b"HORUS\0");
    }

    // ---- Topic types metadata test ----

    #[test]
    fn test_topic_types_metadata() {
        let mut rec = NodeRecording::new("n", "id", "s");
        rec.set_topic_type("sensor.imu", "sensor_msgs/Imu");
        rec.set_topic_type("cmd_vel", "geometry_msgs/Twist");
        // Second set for same topic should not overwrite
        rec.set_topic_type("sensor.imu", "WRONG");

        assert_eq!(
            rec.topic_types.get("sensor.imu").unwrap(),
            "sensor_msgs/Imu"
        );
        assert_eq!(
            rec.topic_types.get("cmd_vel").unwrap(),
            "geometry_msgs/Twist"
        );
    }

    #[test]
    fn test_topic_types_serde_default() {
        // Simulate loading a legacy recording without topic_types
        let dir = tempdir().unwrap();
        let path = dir.path().join("legacy.horus");

        let mut rec = NodeRecording::new("n", "id", "s");
        rec.add_snapshot(NodeTickSnapshot::new(0));
        rec.save(&path).unwrap();

        let loaded = NodeRecording::load(&path).unwrap();
        assert!(loaded.topic_types.is_empty());
    }

    // ── Compression tests ───────────────────────────────────────────────

    #[cfg(feature = "recording-compression")]
    #[test]
    fn test_zstd_save_load_roundtrip() {
        let dir = tempdir().unwrap();
        let path = dir.path().join("compressed.horus");

        let mut rec = NodeRecording::new("sensor", "s1", "sess");
        for i in 0..50 {
            let mut snap = NodeTickSnapshot::new(i);
            snap.outputs.insert("lidar".into(), vec![i as u8; 128]);
            rec.add_snapshot(snap);
        }
        rec.save_with_compression(&path, true).unwrap();

        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.snapshots.len(), 50);
        assert_eq!(loaded.node_name, "sensor");
        assert_eq!(loaded.snapshots[49].outputs["lidar"].len(), 128);
    }

    #[cfg(feature = "recording-compression")]
    #[test]
    fn test_zstd_file_smaller_than_uncompressed() {
        let dir = tempdir().unwrap();
        let plain = dir.path().join("plain.horus");
        let zstd = dir.path().join("zstd.horus");

        let mut rec = NodeRecording::new("node", "id", "sess");
        for i in 0..200 {
            let mut snap = NodeTickSnapshot::new(i);
            // Highly compressible repeated data
            snap.outputs.insert("data".into(), vec![42u8; 1024]);
            rec.add_snapshot(snap);
        }
        rec.save_with_compression(&plain, false).unwrap();
        rec.save_with_compression(&zstd, true).unwrap();

        let plain_size = std::fs::metadata(&plain).unwrap().len();
        let zstd_size = std::fs::metadata(&zstd).unwrap().len();
        assert!(
            zstd_size < plain_size,
            "compressed {} should be < uncompressed {}",
            zstd_size,
            plain_size
        );
    }

    #[cfg(feature = "recording-compression")]
    #[test]
    fn test_zstd_version_header() {
        use std::io::Read;
        let dir = tempdir().unwrap();
        let path = dir.path().join("v2.horus");

        let mut rec = NodeRecording::new("n", "id", "s");
        rec.add_snapshot(NodeTickSnapshot::new(0));
        rec.save_with_compression(&path, true).unwrap();

        let mut f = File::open(&path).unwrap();
        let mut magic = [0u8; 6];
        f.read_exact(&mut magic).unwrap();
        assert_eq!(&magic, RECORDING_MAGIC);
        let mut ver = [0u8; 4];
        f.read_exact(&mut ver).unwrap();
        assert_eq!(u32::from_le_bytes(ver), 2);
    }

    #[test]
    fn test_uncompressed_save_ignores_compress_flag_without_feature() {
        // When feature is off, compress=true falls through to uncompressed
        let dir = tempdir().unwrap();
        let path = dir.path().join("fallback.horus");

        let mut rec = NodeRecording::new("n", "id", "s");
        rec.add_snapshot(NodeTickSnapshot::new(0));
        rec.save_with_compression(&path, true).unwrap();

        // Should load fine as version 1
        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.snapshots.len(), 1);
    }

    #[cfg(feature = "recording-compression")]
    #[test]
    fn test_recorder_finish_uses_compression() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "sess".into(),
            base_dir: dir.path().to_path_buf(),
            interval: 1,
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
            max_size_bytes: 0,
            compress: true,
            max_snapshots: 0,
        };

        let mut recorder = NodeRecorder::new("node", "id", config);
        recorder.begin_tick(0);
        recorder.record_output("topic", vec![1, 2, 3]);
        recorder.end_tick(0);
        let path = recorder.finish().unwrap();

        // Verify it wrote version 2
        use std::io::Read;
        let mut f = File::open(&path).unwrap();
        let mut magic = [0u8; 6];
        f.read_exact(&mut magic).unwrap();
        assert_eq!(&magic, RECORDING_MAGIC);
        let mut ver = [0u8; 4];
        f.read_exact(&mut ver).unwrap();
        assert_eq!(u32::from_le_bytes(ver), 2);

        // Verify it loads correctly
        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.snapshots.len(), 1);
    }

    // ── Concurrent / stress tests ───────────────────────────────────────

    #[test]
    fn test_concurrent_recorders() {
        use std::sync::{Arc, Barrier};

        let dir = tempdir().unwrap();
        let barrier = Arc::new(Barrier::new(4));
        let mut handles = Vec::new();

        for i in 0..4 {
            let dir = dir.path().to_path_buf();
            let barrier = Arc::clone(&barrier);
            handles.push(std::thread::spawn(move || {
                let config = RecordingConfig {
                    session_name: "stress".into(),
                    base_dir: dir,
                    interval: 1,
                    record_inputs: true,
                    record_outputs: true,
                    record_timing: true,
                    max_size_bytes: 0,
                    compress: false,
                    max_snapshots: 0,
                };

                let mut recorder =
                    NodeRecorder::new(&format!("node_{}", i), &format!("id_{}", i), config);

                // Synchronize start
                barrier.wait();

                for tick in 0..100 {
                    recorder.begin_tick(tick);
                    recorder.record_output("data", vec![tick as u8; 64]);
                    recorder.end_tick(tick);
                }

                recorder.finish().unwrap()
            }));
        }

        let paths: Vec<_> = handles.into_iter().map(|h| h.join().unwrap()).collect();
        // All files should exist and be loadable
        for path in &paths {
            let rec = NodeRecording::load(path).unwrap();
            assert_eq!(rec.snapshots.len(), 100);
        }
        // All unique files
        let unique: std::collections::HashSet<_> = paths.iter().collect();
        assert_eq!(unique.len(), 4);
    }

    #[test]
    fn test_high_frequency_recording() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "highfreq".into(),
            base_dir: dir.path().to_path_buf(),
            interval: 1,
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
            max_size_bytes: 0,
            compress: false,
            max_snapshots: 0,
        };

        let mut recorder = NodeRecorder::new("fast_node", "f1", config);

        // Simulate 1000 ticks at high frequency
        for tick in 0..1000 {
            recorder.begin_tick(tick);
            recorder.record_output("imu", vec![0u8; 48]); // typical IMU frame
            recorder.record_input("cmd", vec![0u8; 16]); // command input
            recorder.end_tick(tick);
        }

        let path = recorder.finish().unwrap();
        let loaded = NodeRecording::load(&path).unwrap();
        assert_eq!(loaded.snapshots.len(), 1000);

        // Verify all ticks are sequential
        for (i, snap) in loaded.snapshots.iter().enumerate() {
            assert_eq!(snap.tick, i as u64);
        }
    }

    #[test]
    fn test_ring_buffer_under_stress() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "ring_stress".into(),
            base_dir: dir.path().to_path_buf(),
            interval: 1,
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
            max_size_bytes: 0,
            compress: false,
            max_snapshots: 50, // keep last 50
        };

        let mut recorder = NodeRecorder::new("ring_node", "r1", config);

        for tick in 0..500 {
            recorder.begin_tick(tick);
            recorder.record_output("data", vec![tick as u8; 32]);
            recorder.end_tick(tick);
        }

        let path = recorder.finish().unwrap();
        let loaded = NodeRecording::load(&path).unwrap();

        // Should have exactly max_snapshots
        assert_eq!(loaded.snapshots.len(), 50);
        // Should contain the LAST 50 ticks
        assert_eq!(loaded.snapshots[0].tick, 450);
        assert_eq!(loaded.snapshots[49].tick, 499);
    }

    #[test]
    fn test_size_limit_stops_recording_stress() {
        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "size_limit".into(),
            base_dir: dir.path().to_path_buf(),
            interval: 1,
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
            max_size_bytes: 1024, // very small limit
            compress: false,
            max_snapshots: 0,
        };

        let mut recorder = NodeRecorder::new("limited", "l1", config);

        for tick in 0..1000 {
            recorder.begin_tick(tick);
            recorder.record_output("big", vec![0u8; 256]);
            recorder.end_tick(tick);
        }

        let path = recorder.finish().unwrap();
        let loaded = NodeRecording::load(&path).unwrap();
        // Should have stopped well before 1000
        assert!(
            loaded.snapshots.len() < 100,
            "got {} snapshots, expected < 100",
            loaded.snapshots.len()
        );
        assert!(!loaded.snapshots.is_empty());
    }

    #[test]
    fn test_replayer_advance_full_sequence() {
        let mut recording = NodeRecording::new("seq", "s1", "sess");
        for i in 0..10 {
            let mut snap = NodeTickSnapshot::new(i);
            snap.outputs.insert("val".into(), vec![i as u8]);
            recording.add_snapshot(snap);
        }

        let mut replayer = NodeReplayer::from_recording(recording);
        let mut ticks_seen = Vec::new();
        loop {
            ticks_seen.push(replayer.current_tick());
            if !replayer.advance() {
                break;
            }
        }
        assert_eq!(ticks_seen.len(), 10);
        assert_eq!(ticks_seen[0], 0);
        assert_eq!(ticks_seen[9], 9);
    }

    // ── Performance benchmark ───────────────────────────────────────────

    #[test]
    fn test_recording_overhead_benchmark() {
        use std::time::Instant;

        let dir = tempdir().unwrap();
        let config = RecordingConfig {
            session_name: "bench".into(),
            base_dir: dir.path().to_path_buf(),
            interval: 1,
            record_inputs: true,
            record_outputs: true,
            record_timing: true,
            max_size_bytes: 0,
            compress: false,
            max_snapshots: 0,
        };

        let mut recorder = NodeRecorder::new("bench_node", "b1", config);
        let payload = vec![0u8; 256]; // typical sensor payload

        // Warm up
        for tick in 0..100 {
            recorder.begin_tick(tick);
            recorder.record_output("sensor", payload.clone());
            recorder.end_tick(tick);
        }

        // Benchmark 10000 ticks
        let start = Instant::now();
        let bench_ticks = 10_000u64;
        for tick in 100..(100 + bench_ticks) {
            recorder.begin_tick(tick);
            recorder.record_output("sensor", payload.clone());
            recorder.record_input("cmd", vec![0u8; 16]);
            recorder.end_tick(tick);
        }
        let elapsed = start.elapsed();
        let per_tick_us = elapsed.as_micros() as f64 / bench_ticks as f64;

        // At 1kHz each tick budget is 1000us; recording overhead should be < 50us
        assert!(
            per_tick_us < 50.0,
            "Recording overhead {:.1}us/tick exceeds 50us budget",
            per_tick_us
        );

        // Save/load benchmark
        let save_start = Instant::now();
        let path = recorder.finish().unwrap();
        let save_elapsed = save_start.elapsed();

        let load_start = Instant::now();
        let loaded = NodeRecording::load(&path).unwrap();
        let load_elapsed = load_start.elapsed();

        assert_eq!(loaded.snapshots.len(), 10_100);
        // Save + load of ~10k snapshots should be < 2s
        assert!(
            save_elapsed.as_secs() < 2,
            "Save took {:.1}s, expected < 2s",
            save_elapsed.as_secs_f64()
        );
        assert!(
            load_elapsed.as_secs() < 2,
            "Load took {:.1}s, expected < 2s",
            load_elapsed.as_secs_f64()
        );
    }

    #[test]
    fn test_replayer_seek_performance() {
        let mut recording = NodeRecording::new("perf", "p1", "sess");
        for i in 0..5000 {
            let mut snap = NodeTickSnapshot::new(i);
            snap.outputs.insert("data".into(), vec![i as u8; 64]);
            recording.add_snapshot(snap);
        }

        let mut replayer = NodeReplayer::from_recording(recording);

        // Seek to various positions — should be near-instant
        let start = std::time::Instant::now();
        for target in [0, 2500, 4999, 1000, 3000, 0, 4999] {
            replayer.seek(target);
            assert_eq!(replayer.current_tick(), target);
        }
        let elapsed = start.elapsed();
        assert!(
            elapsed.as_millis() < 100,
            "7 seeks over 5000 snapshots took {}ms, expected < 100ms",
            elapsed.as_millis()
        );
    }

    // ---- Battle tests: debugger breakpoint flow ----

    #[test]
    fn test_debugger_multiple_breakpoints_hit_in_order() {
        let rec = make_test_recording(20);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        let bp1 = dbg.add_breakpoint(BreakpointCondition::AtTick(5));
        let bp2 = dbg.add_breakpoint(BreakpointCondition::AtTick(15));

        // First continue: should stop at tick 5
        dbg.continue_execution();
        assert_eq!(dbg.current_tick(), 5, "should stop at first breakpoint");
        assert_eq!(
            dbg.breakpoints()
                .iter()
                .find(|b| b.id == bp1)
                .unwrap()
                .hit_count,
            1,
            "bp1 should be hit once"
        );

        // Second continue: should stop at tick 15
        dbg.continue_execution();
        assert_eq!(dbg.current_tick(), 15, "should stop at second breakpoint");
        assert_eq!(
            dbg.breakpoints()
                .iter()
                .find(|b| b.id == bp2)
                .unwrap()
                .hit_count,
            1,
            "bp2 should be hit once"
        );
    }

    #[test]
    fn test_debugger_continue_past_disabled_breakpoint() {
        let rec = make_test_recording(10);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        let bp1 = dbg.add_breakpoint(BreakpointCondition::AtTick(3));
        let _bp2 = dbg.add_breakpoint(BreakpointCondition::AtTick(7));

        // Hit first breakpoint
        dbg.continue_execution();
        assert_eq!(dbg.current_tick(), 3);

        // Disable first breakpoint and continue — should hit second
        dbg.set_breakpoint_enabled(bp1, false);
        dbg.continue_execution();
        assert_eq!(dbg.current_tick(), 7, "should skip disabled bp and hit bp2");
    }

    #[test]
    fn test_debugger_step_then_continue_to_breakpoint() {
        let rec = make_test_recording(10);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        dbg.add_breakpoint(BreakpointCondition::AtTick(5));

        // Manual step twice
        dbg.step_forward(); // tick 1
        dbg.step_forward(); // tick 2
        assert_eq!(dbg.current_tick(), 2);

        // Now continue — should hit breakpoint at tick 5
        dbg.continue_execution();
        assert_eq!(
            dbg.current_tick(),
            5,
            "continue from tick 2 should hit bp at 5"
        );
    }

    #[test]
    fn test_debugger_events_contain_breakpoint_hit() {
        let rec = make_test_recording(10);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        dbg.add_breakpoint(BreakpointCondition::AtTick(3));
        dbg.continue_execution();

        let events = dbg.recent_events(20);
        assert!(
            !events.is_empty(),
            "events should contain entries after breakpoint hit"
        );
        // Verify at least one event mentions the breakpoint or tick
        let event_strs: Vec<String> = events.iter().map(|e| format!("{:?}", e)).collect();
        let has_breakpoint_event = event_strs
            .iter()
            .any(|s| s.contains("Breakpoint") || s.contains("breakpoint") || s.contains("Hit"));
        assert!(
            has_breakpoint_event,
            "events should contain breakpoint hit event. Events: {:?}",
            event_strs
        );
    }

    #[test]
    fn test_debugger_evaluate_watches_at_tick() {
        let rec = make_test_recording(10);
        let replayer = NodeReplayer::from_recording(rec);
        let mut dbg = ReplayDebugger::new(replayer);

        // Add watch on "motor" output
        dbg.add_watch(WatchExpression::output("w1", "Motor Output", "motor"));

        // Step to tick 3
        dbg.seek(3);
        assert_eq!(dbg.current_tick(), 3);

        let values = dbg.evaluate_watches();
        assert!(!values.is_empty(), "watch should have a value at tick 3");
        let v = &values[0];
        assert_eq!(v.expression_id, "w1");
        assert_eq!(v.tick, 3);
        // motor output at tick 3 = vec![(3*2) as u8; 8] = vec![6; 8]
        assert_eq!(
            v.raw_bytes,
            vec![6u8; 8],
            "motor output at tick 3 should be [6; 8]"
        );
    }
}
