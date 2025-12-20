//! Distributed Recording System
//!
//! Provides coordinated recording across multiple processes/robots for:
//! - Consistent snapshots across distributed systems
//! - Vector clocks for causality tracking
//! - Cross-process replay coordination
//! - Fleet-wide recording and analysis
//!
//! ## Architecture
//!
//! ```text
//! ┌─────────────────────────────────────────────────────────────────────────┐
//! │                     Distributed Recording                               │
//! ├─────────────────────────────────────────────────────────────────────────┤
//! │                                                                          │
//! │  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐                  │
//! │  │  Process A   │  │  Process B   │  │  Process C   │                  │
//! │  │  (Robot 1)   │  │  (Robot 2)   │  │  (Server)    │                  │
//! │  └──────┬───────┘  └──────┬───────┘  └──────┬───────┘                  │
//! │         │                  │                  │                          │
//! │         ▼                  ▼                  ▼                          │
//! │  ┌────────────────────────────────────────────────────────────────┐    │
//! │  │              Recording Coordinator (Shared Memory / IPC)        │    │
//! │  │  - Vector Clock Synchronization                                 │    │
//! │  │  - Snapshot Barriers                                            │    │
//! │  │  - Timeline Merging                                             │    │
//! │  └────────────────────────────────────────────────────────────────┘    │
//! │                              │                                          │
//! │                              ▼                                          │
//! │  ┌────────────────────────────────────────────────────────────────┐    │
//! │  │              Unified Recording (Merged Timeline)                │    │
//! │  │  - Causally ordered events                                      │    │
//! │  │  - Per-process recordings                                       │    │
//! │  │  - Cross-process dependencies                                   │    │
//! │  └────────────────────────────────────────────────────────────────┘    │
//! └─────────────────────────────────────────────────────────────────────────┘
//! ```

use std::collections::{BTreeMap, HashMap};
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use std::time::{Duration, Instant, SystemTime, UNIX_EPOCH};

use parking_lot::{Mutex, RwLock};
use serde::{Deserialize, Serialize};
use thiserror::Error;

use super::zero_copy_recording::{StreamingRecorder, ZeroCopyError, ZeroCopyRecording};

/// Errors in distributed recording
#[derive(Error, Debug)]
pub enum DistributedError {
    #[error("Zero-copy error: {0}")]
    ZeroCopy(#[from] ZeroCopyError),

    #[error("IO error: {0}")]
    Io(#[from] std::io::Error),

    #[error("Coordinator not found")]
    CoordinatorNotFound,

    #[error("Process not registered: {0}")]
    ProcessNotRegistered(String),

    #[error("Snapshot barrier timeout")]
    BarrierTimeout,

    #[error("Recording not started")]
    RecordingNotStarted,

    #[error("Causality violation: {0}")]
    CausalityViolation(String),

    #[error("Serialization error: {0}")]
    SerializationError(String),
}

pub type Result<T> = std::result::Result<T, DistributedError>;

// ============================================================================
// Vector Clock
// ============================================================================

/// Vector clock for distributed causality tracking
#[derive(Debug, Clone, Default, Serialize, Deserialize, PartialEq, Eq)]
pub struct VectorClock {
    /// Process ID -> logical clock value
    clocks: BTreeMap<String, u64>,
}

impl VectorClock {
    /// Create a new vector clock
    pub fn new() -> Self {
        Self {
            clocks: BTreeMap::new(),
        }
    }

    /// Increment the clock for a process
    pub fn increment(&mut self, process_id: &str) {
        *self.clocks.entry(process_id.to_string()).or_insert(0) += 1;
    }

    /// Get the clock value for a process
    pub fn get(&self, process_id: &str) -> u64 {
        self.clocks.get(process_id).copied().unwrap_or(0)
    }

    /// Merge with another vector clock (take max of each component)
    pub fn merge(&mut self, other: &VectorClock) {
        for (process, &clock) in &other.clocks {
            let entry = self.clocks.entry(process.clone()).or_insert(0);
            *entry = (*entry).max(clock);
        }
    }

    /// Check if this clock happened-before another
    pub fn happened_before(&self, other: &VectorClock) -> bool {
        let mut dominated = false;

        for (process, &self_clock) in &self.clocks {
            let other_clock = other.get(process);
            if self_clock > other_clock {
                return false; // Not dominated
            }
            if self_clock < other_clock {
                dominated = true;
            }
        }

        // Check for any process in other that's not in self
        for (process, &other_clock) in &other.clocks {
            if !self.clocks.contains_key(process) && other_clock > 0 {
                dominated = true;
            }
        }

        dominated
    }

    /// Check if two clocks are concurrent (neither happened-before the other)
    pub fn concurrent(&self, other: &VectorClock) -> bool {
        !self.happened_before(other) && !other.happened_before(self) && self != other
    }

    /// Serialize to bytes
    pub fn to_bytes(&self) -> Vec<u8> {
        bincode::serialize(self).unwrap_or_default()
    }

    /// Deserialize from bytes
    pub fn from_bytes(bytes: &[u8]) -> Option<Self> {
        bincode::deserialize(bytes).ok()
    }
}

// ============================================================================
// Distributed Event
// ============================================================================

/// A distributed event with causality information
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct DistributedEvent {
    /// Unique event ID
    pub id: u64,
    /// Process that generated this event
    pub process_id: String,
    /// Local sequence number
    pub sequence: u64,
    /// Vector clock at event time
    pub vector_clock: VectorClock,
    /// Wall clock timestamp (nanoseconds since epoch)
    pub timestamp_ns: u64,
    /// Event type
    pub event_type: DistributedEventType,
    /// Payload data
    pub data: Vec<u8>,
}

/// Type of distributed event
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum DistributedEventType {
    /// Message sent
    Send,
    /// Message received
    Receive,
    /// Local computation
    Local,
    /// Snapshot marker
    Snapshot,
    /// Barrier sync
    Barrier,
    /// Custom event
    Custom,
}

// ============================================================================
// Process Recorder
// ============================================================================

/// Per-process distributed recorder
pub struct ProcessRecorder {
    /// Process identifier
    process_id: String,
    /// Local sequence counter
    sequence: AtomicU64,
    /// Local vector clock
    clock: RwLock<VectorClock>,
    /// Events buffer
    events: Mutex<Vec<DistributedEvent>>,
    /// Underlying zero-copy recorder
    zc_recorder: Option<Mutex<StreamingRecorder>>,
    /// Recording flag
    recording: AtomicBool,
    /// Next event ID
    next_event_id: AtomicU64,
}

impl ProcessRecorder {
    /// Create a new process recorder
    pub fn new(process_id: &str) -> Self {
        Self {
            process_id: process_id.to_string(),
            sequence: AtomicU64::new(0),
            clock: RwLock::new(VectorClock::new()),
            events: Mutex::new(Vec::new()),
            zc_recorder: None,
            recording: AtomicBool::new(false),
            next_event_id: AtomicU64::new(0),
        }
    }

    /// Start recording with zero-copy backend
    pub fn start_recording(&mut self, session_id: &str) -> Result<()> {
        let recorder = StreamingRecorder::with_node_name(session_id, Some(&self.process_id))?;
        self.zc_recorder = Some(Mutex::new(recorder));
        self.recording.store(true, Ordering::Release);
        Ok(())
    }

    /// Record a local event
    pub fn record_local(&self, data: &[u8]) -> Result<DistributedEvent> {
        self.record_event(DistributedEventType::Local, data, None)
    }

    /// Record a send event
    pub fn record_send(&self, data: &[u8]) -> Result<DistributedEvent> {
        self.record_event(DistributedEventType::Send, data, None)
    }

    /// Record a receive event (merges clock from sender)
    pub fn record_receive(
        &self,
        data: &[u8],
        sender_clock: &VectorClock,
    ) -> Result<DistributedEvent> {
        self.record_event(DistributedEventType::Receive, data, Some(sender_clock))
    }

    /// Record an event
    fn record_event(
        &self,
        event_type: DistributedEventType,
        data: &[u8],
        remote_clock: Option<&VectorClock>,
    ) -> Result<DistributedEvent> {
        // Update vector clock
        {
            let mut clock = self.clock.write();
            if let Some(remote) = remote_clock {
                clock.merge(remote);
            }
            clock.increment(&self.process_id);
        }

        let event = DistributedEvent {
            id: self.next_event_id.fetch_add(1, Ordering::Relaxed),
            process_id: self.process_id.clone(),
            sequence: self.sequence.fetch_add(1, Ordering::Relaxed),
            vector_clock: self.clock.read().clone(),
            timestamp_ns: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            event_type,
            data: data.to_vec(),
        };

        // Store event
        self.events.lock().push(event.clone());

        // Record to zero-copy backend if active
        if self.recording.load(Ordering::Acquire) {
            if let Some(ref recorder) = self.zc_recorder {
                let mut rec = recorder.lock();
                let _ = rec.begin_tick(event.sequence);
                let event_bytes = bincode::serialize(&event)
                    .map_err(|e| DistributedError::SerializationError(e.to_string()))?;
                let _ = rec.record_raw("distributed_events", event_type as u8, &event_bytes);
            }
        }

        Ok(event)
    }

    /// Get current vector clock
    pub fn current_clock(&self) -> VectorClock {
        self.clock.read().clone()
    }

    /// Get all recorded events
    pub fn events(&self) -> Vec<DistributedEvent> {
        self.events.lock().clone()
    }

    /// Finalize recording
    pub fn finalize(self) -> Result<Option<ZeroCopyRecording>> {
        self.recording.store(false, Ordering::Release);
        if let Some(recorder) = self.zc_recorder {
            let rec = recorder.into_inner();
            Ok(Some(rec.finalize()?))
        } else {
            Ok(None)
        }
    }
}

// ============================================================================
// Recording Coordinator
// ============================================================================

/// Coordinates recording across multiple processes
pub struct RecordingCoordinator {
    /// Session identifier
    session_id: String,
    /// Registered processes
    processes: RwLock<HashMap<String, ProcessInfo>>,
    /// Global event counter
    #[allow(dead_code)] // Reserved for future event sequencing
    global_event_counter: AtomicU64,
    /// Barrier state
    barrier: Mutex<BarrierState>,
    /// Recording active
    #[allow(dead_code)] // Reserved for future active state tracking
    active: AtomicBool,
    /// Base directory for recordings
    base_dir: PathBuf,
}

/// Information about a registered process
#[derive(Debug, Clone)]
#[allow(dead_code)] // Fields reserved for future process monitoring
struct ProcessInfo {
    id: String,
    registered_at: Instant,
    last_heartbeat: Instant,
    event_count: u64,
}

/// State for snapshot barriers
#[derive(Debug, Default)]
struct BarrierState {
    /// Processes that have reached the barrier
    arrived: HashMap<String, VectorClock>,
    /// Total processes expected
    expected: usize,
    /// Barrier generation
    generation: u64,
}

impl RecordingCoordinator {
    /// Create a new recording coordinator
    pub fn new(session_id: &str) -> Result<Self> {
        let base_dir = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(".horus")
            .join("recordings")
            .join("distributed")
            .join(session_id);

        std::fs::create_dir_all(&base_dir)?;

        Ok(Self {
            session_id: session_id.to_string(),
            processes: RwLock::new(HashMap::new()),
            global_event_counter: AtomicU64::new(0),
            barrier: Mutex::new(BarrierState::default()),
            active: AtomicBool::new(false),
            base_dir,
        })
    }

    /// Register a process with the coordinator
    pub fn register_process(&self, process_id: &str) -> Result<ProcessRecorder> {
        let mut processes = self.processes.write();

        let info = ProcessInfo {
            id: process_id.to_string(),
            registered_at: Instant::now(),
            last_heartbeat: Instant::now(),
            event_count: 0,
        };

        processes.insert(process_id.to_string(), info);

        let mut recorder = ProcessRecorder::new(process_id);

        if self.active.load(Ordering::Acquire) {
            recorder.start_recording(&self.session_id)?;
        }

        Ok(recorder)
    }

    /// Start distributed recording
    pub fn start_recording(&self) -> Result<()> {
        self.active.store(true, Ordering::Release);
        Ok(())
    }

    /// Stop distributed recording
    pub fn stop_recording(&self) {
        self.active.store(false, Ordering::Release);
    }

    /// Initiate a coordinated snapshot (Chandy-Lamport style)
    pub fn initiate_snapshot(&self) -> Result<SnapshotResult> {
        let processes = self.processes.read();
        let process_ids: Vec<String> = processes.keys().cloned().collect();
        drop(processes);

        // Set up barrier
        {
            let mut barrier = self.barrier.lock();
            barrier.arrived.clear();
            barrier.expected = process_ids.len();
            barrier.generation += 1;
        }

        // Return snapshot request that processes should honor
        Ok(SnapshotResult {
            generation: self.barrier.lock().generation,
            processes: process_ids,
            timestamp: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
        })
    }

    /// Process arrives at snapshot barrier
    pub fn arrive_at_barrier(&self, process_id: &str, clock: VectorClock) -> Result<bool> {
        let mut barrier = self.barrier.lock();

        barrier.arrived.insert(process_id.to_string(), clock);

        Ok(barrier.arrived.len() >= barrier.expected)
    }

    /// Wait for all processes to arrive at barrier
    pub fn wait_for_barrier(&self, timeout: Duration) -> Result<HashMap<String, VectorClock>> {
        let start = Instant::now();

        loop {
            {
                let barrier = self.barrier.lock();
                if barrier.arrived.len() >= barrier.expected {
                    return Ok(barrier.arrived.clone());
                }
            }

            if start.elapsed() > timeout {
                return Err(DistributedError::BarrierTimeout);
            }

            std::thread::sleep(Duration::from_millis(10));
        }
    }

    /// Get the merged timeline from all processes
    pub fn merge_timelines(
        &self,
        process_events: &[Vec<DistributedEvent>],
    ) -> Vec<DistributedEvent> {
        // Collect all events
        let mut all_events: Vec<DistributedEvent> = process_events
            .iter()
            .flat_map(|events| events.iter().cloned())
            .collect();

        // Sort by causality (topological sort based on vector clocks)
        // Events are ordered by:
        // 1. Happened-before relationship
        // 2. Wall clock time (for concurrent events)
        all_events.sort_by(|a, b| {
            if a.vector_clock.happened_before(&b.vector_clock) {
                std::cmp::Ordering::Less
            } else if b.vector_clock.happened_before(&a.vector_clock) {
                std::cmp::Ordering::Greater
            } else {
                // Concurrent events - use wall clock
                a.timestamp_ns.cmp(&b.timestamp_ns)
            }
        });

        all_events
    }

    /// Export merged recording to file
    pub fn export_merged(&self, events: &[DistributedEvent]) -> Result<PathBuf> {
        let output_path = self.base_dir.join("merged_timeline.json");
        let file = std::fs::File::create(&output_path)?;
        serde_json::to_writer_pretty(file, events)
            .map_err(|e| DistributedError::SerializationError(e.to_string()))?;
        Ok(output_path)
    }
}

/// Result of initiating a snapshot
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct SnapshotResult {
    pub generation: u64,
    pub processes: Vec<String>,
    pub timestamp: u64,
}

// ============================================================================
// Fleet Recording
// ============================================================================

/// Coordinates recording across multiple robots
pub struct FleetRecorder {
    /// Fleet session ID
    session_id: String,
    /// Robot recordings
    robots: RwLock<HashMap<String, RobotRecording>>,
    /// Global timeline
    global_timeline: RwLock<Vec<FleetEvent>>,
    /// Fleet-wide vector clock
    fleet_clock: RwLock<VectorClock>,
    /// Active flag
    #[allow(dead_code)] // Reserved for future active state tracking
    active: AtomicBool,
    /// Base directory
    base_dir: PathBuf,
}

/// Recording for a single robot
#[derive(Debug, Clone)]
pub struct RobotRecording {
    pub robot_id: String,
    pub process_recorders: Vec<String>, // Process IDs
    pub events: Vec<DistributedEvent>,
    pub start_time: u64,
    pub end_time: Option<u64>,
}

/// A fleet-wide event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetEvent {
    /// Source robot
    pub robot_id: String,
    /// Original event
    pub event: DistributedEvent,
    /// Fleet-wide sequence number
    pub fleet_sequence: u64,
    /// Cross-robot dependencies
    pub dependencies: Vec<FleetEventRef>,
}

/// Reference to another fleet event
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct FleetEventRef {
    pub robot_id: String,
    pub event_id: u64,
}

impl FleetRecorder {
    /// Create a new fleet recorder
    pub fn new(session_id: &str) -> Result<Self> {
        let base_dir = dirs::home_dir()
            .unwrap_or_else(|| PathBuf::from("."))
            .join(".horus")
            .join("recordings")
            .join("fleet")
            .join(session_id);

        std::fs::create_dir_all(&base_dir)?;

        Ok(Self {
            session_id: session_id.to_string(),
            robots: RwLock::new(HashMap::new()),
            global_timeline: RwLock::new(Vec::new()),
            fleet_clock: RwLock::new(VectorClock::new()),
            active: AtomicBool::new(false),
            base_dir,
        })
    }

    /// Register a robot with the fleet
    pub fn register_robot(&self, robot_id: &str) -> Result<()> {
        let mut robots = self.robots.write();

        let recording = RobotRecording {
            robot_id: robot_id.to_string(),
            process_recorders: Vec::new(),
            events: Vec::new(),
            start_time: SystemTime::now()
                .duration_since(UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64,
            end_time: None,
        };

        robots.insert(robot_id.to_string(), recording);
        Ok(())
    }

    /// Record an event from a robot
    pub fn record_event(&self, robot_id: &str, event: DistributedEvent) -> Result<u64> {
        // Update fleet clock
        let fleet_sequence = {
            let mut clock = self.fleet_clock.write();
            clock.merge(&event.vector_clock);
            clock.increment(robot_id);
            clock.get(robot_id)
        };

        // Create fleet event
        let fleet_event = FleetEvent {
            robot_id: robot_id.to_string(),
            event: event.clone(),
            fleet_sequence,
            dependencies: Vec::new(), // Could be computed from vector clock
        };

        // Add to timeline
        self.global_timeline.write().push(fleet_event);

        // Add to robot's events
        if let Some(robot) = self.robots.write().get_mut(robot_id) {
            robot.events.push(event);
        }

        Ok(fleet_sequence)
    }

    /// Get the global fleet timeline
    pub fn timeline(&self) -> Vec<FleetEvent> {
        self.global_timeline.read().clone()
    }

    /// Export fleet recording
    pub fn export(&self) -> Result<PathBuf> {
        let output_path = self.base_dir.join("fleet_recording.json");

        let data = FleetRecordingExport {
            session_id: self.session_id.clone(),
            robots: self.robots.read().clone(),
            timeline: self.global_timeline.read().clone(),
            fleet_clock: self.fleet_clock.read().clone(),
        };

        let file = std::fs::File::create(&output_path)?;
        serde_json::to_writer_pretty(file, &data)
            .map_err(|e| DistributedError::SerializationError(e.to_string()))?;

        Ok(output_path)
    }

    /// Replay a specific robot while others are live
    pub fn hybrid_replay_mode(&self, replay_robot: &str) -> HybridReplayConfig {
        HybridReplayConfig {
            replay_robot: replay_robot.to_string(),
            live_robots: self
                .robots
                .read()
                .keys()
                .filter(|r| *r != replay_robot)
                .cloned()
                .collect(),
            sync_on_messages: true,
        }
    }
}

/// Export format for fleet recording
#[derive(Debug, Clone, Serialize, Deserialize)]
struct FleetRecordingExport {
    session_id: String,
    robots: HashMap<String, RobotRecording>,
    timeline: Vec<FleetEvent>,
    fleet_clock: VectorClock,
}

/// Configuration for hybrid replay (some robots replay, some live)
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct HybridReplayConfig {
    pub replay_robot: String,
    pub live_robots: Vec<String>,
    pub sync_on_messages: bool,
}

impl Serialize for RobotRecording {
    fn serialize<S>(&self, serializer: S) -> std::result::Result<S::Ok, S::Error>
    where
        S: serde::Serializer,
    {
        use serde::ser::SerializeStruct;
        let mut state = serializer.serialize_struct("RobotRecording", 5)?;
        state.serialize_field("robot_id", &self.robot_id)?;
        state.serialize_field("process_recorders", &self.process_recorders)?;
        state.serialize_field("events", &self.events)?;
        state.serialize_field("start_time", &self.start_time)?;
        state.serialize_field("end_time", &self.end_time)?;
        state.end()
    }
}

impl<'de> Deserialize<'de> for RobotRecording {
    fn deserialize<D>(deserializer: D) -> std::result::Result<Self, D::Error>
    where
        D: serde::Deserializer<'de>,
    {
        #[derive(Deserialize)]
        struct Helper {
            robot_id: String,
            process_recorders: Vec<String>,
            events: Vec<DistributedEvent>,
            start_time: u64,
            end_time: Option<u64>,
        }

        let helper = Helper::deserialize(deserializer)?;
        Ok(RobotRecording {
            robot_id: helper.robot_id,
            process_recorders: helper.process_recorders,
            events: helper.events,
            start_time: helper.start_time,
            end_time: helper.end_time,
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
    fn test_vector_clock_basic() {
        let mut clock = VectorClock::new();

        clock.increment("A");
        assert_eq!(clock.get("A"), 1);
        assert_eq!(clock.get("B"), 0);

        clock.increment("A");
        clock.increment("B");
        assert_eq!(clock.get("A"), 2);
        assert_eq!(clock.get("B"), 1);
    }

    #[test]
    fn test_vector_clock_merge() {
        let mut clock1 = VectorClock::new();
        clock1.increment("A");
        clock1.increment("A");

        let mut clock2 = VectorClock::new();
        clock2.increment("B");
        clock2.increment("B");
        clock2.increment("B");

        clock1.merge(&clock2);

        assert_eq!(clock1.get("A"), 2);
        assert_eq!(clock1.get("B"), 3);
    }

    #[test]
    fn test_vector_clock_happened_before() {
        let mut clock1 = VectorClock::new();
        clock1.increment("A");

        let mut clock2 = clock1.clone();
        clock2.increment("A");

        assert!(clock1.happened_before(&clock2));
        assert!(!clock2.happened_before(&clock1));
    }

    #[test]
    fn test_vector_clock_concurrent() {
        let mut clock1 = VectorClock::new();
        clock1.increment("A");

        let mut clock2 = VectorClock::new();
        clock2.increment("B");

        assert!(clock1.concurrent(&clock2));
        assert!(clock2.concurrent(&clock1));
    }

    #[test]
    fn test_process_recorder() {
        let recorder = ProcessRecorder::new("process_1");

        let event1 = recorder.record_local(&[1, 2, 3]).unwrap();
        assert_eq!(event1.process_id, "process_1");
        assert_eq!(event1.sequence, 0);

        let event2 = recorder.record_local(&[4, 5, 6]).unwrap();
        assert_eq!(event2.sequence, 1);

        let events = recorder.events();
        assert_eq!(events.len(), 2);
    }

    #[test]
    fn test_recording_coordinator() -> Result<()> {
        let session_id = format!("test_coord_{}", uuid::Uuid::new_v4());
        let coordinator = RecordingCoordinator::new(&session_id)?;

        let recorder1 = coordinator.register_process("robot_1")?;
        let recorder2 = coordinator.register_process("robot_2")?;

        // Record some events
        let event1 = recorder1.record_send(&[1, 2, 3])?;
        let event2 = recorder2.record_receive(&[1, 2, 3], &event1.vector_clock)?;

        // Verify causality
        assert!(event1.vector_clock.happened_before(&event2.vector_clock));

        // Cleanup
        std::fs::remove_dir_all(&coordinator.base_dir).ok();

        Ok(())
    }

    #[test]
    fn test_timeline_merge() -> Result<()> {
        let session_id = format!("test_merge_{}", uuid::Uuid::new_v4());
        let coordinator = RecordingCoordinator::new(&session_id)?;

        let recorder1 = coordinator.register_process("A")?;
        let recorder2 = coordinator.register_process("B")?;

        // Create events with causality
        let e1 = recorder1.record_local(&[1])?;
        let e2 = recorder2.record_receive(&[2], &e1.vector_clock)?;
        let _e3 = recorder1.record_receive(&[3], &e2.vector_clock)?;

        let merged = coordinator.merge_timelines(&[recorder1.events(), recorder2.events()]);

        // Verify causal order
        assert_eq!(merged.len(), 3);
        assert!(
            merged[0]
                .vector_clock
                .happened_before(&merged[1].vector_clock)
                || merged[0].vector_clock == merged[1].vector_clock
                || merged[0].timestamp_ns <= merged[1].timestamp_ns
        );

        // Cleanup
        std::fs::remove_dir_all(&coordinator.base_dir).ok();

        Ok(())
    }

    #[test]
    fn test_fleet_recorder() -> Result<()> {
        let session_id = format!("test_fleet_{}", uuid::Uuid::new_v4());
        let fleet = FleetRecorder::new(&session_id)?;

        fleet.register_robot("robot_1")?;
        fleet.register_robot("robot_2")?;

        // Record events
        let event1 = DistributedEvent {
            id: 0,
            process_id: "robot_1".to_string(),
            sequence: 0,
            vector_clock: VectorClock::new(),
            timestamp_ns: 1000,
            event_type: DistributedEventType::Local,
            data: vec![1, 2, 3],
        };

        let seq = fleet.record_event("robot_1", event1)?;
        assert_eq!(seq, 1);

        let timeline = fleet.timeline();
        assert_eq!(timeline.len(), 1);
        assert_eq!(timeline[0].robot_id, "robot_1");

        // Cleanup
        std::fs::remove_dir_all(&fleet.base_dir).ok();

        Ok(())
    }
}
