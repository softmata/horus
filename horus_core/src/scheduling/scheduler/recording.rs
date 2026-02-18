//! Record/Replay System for the Scheduler.
//!
//! Provides methods to record node execution for crash investigation and
//! replay recordings for debugging, time travel, and what-if testing.

use std::path::PathBuf;

use crate::error::HorusResult;
use crate::horus_internal;
use crate::terminal::print_line;

use super::super::intelligence::NodeTier;
use super::super::record_replay::{
    NodeReplayer, RecordingConfig, RecordingManager, ReplayMode, ReplayNode, SchedulerRecording,
};
use super::super::types::RegisteredNode;
use super::Scheduler;

impl Scheduler {
    /// Enable recording for this scheduler session (builder pattern).
    ///
    /// When enabled, all node inputs/outputs are recorded to disk for later replay.
    /// Recordings are saved to `~/.horus/recordings/<session_name>/`.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// let scheduler = Scheduler::new()
    ///     .enable_recording("crash_investigation");  // One line!
    /// ```
    pub fn enable_recording(mut self, session_name: &str) -> Self {
        let config = RecordingConfig::with_name(session_name);
        let scheduler_id = generate_scheduler_id();

        self.scheduler_recording = Some(SchedulerRecording::new(&scheduler_id, session_name));
        self.recording_config = Some(config);

        print_line(&format!(
            "[RECORDING] Enabled for session '{}' (scheduler@{})",
            session_name, scheduler_id
        ));
        self
    }

    /// Enable recording with custom configuration.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::RecordingConfig;
    ///
    /// let config = RecordingConfig {
    ///     session_name: "my_session".to_string(),
    ///     compress: true,
    ///     interval: 1,  // Record every tick
    ///     ..Default::default()
    /// };
    /// let scheduler = Scheduler::new()
    ///     .enable_recording_with_config(config);
    /// ```
    pub fn enable_recording_with_config(mut self, config: RecordingConfig) -> Self {
        let scheduler_id = generate_scheduler_id();
        let session_name = config.session_name.clone();

        self.scheduler_recording = Some(SchedulerRecording::new(&scheduler_id, &session_name));
        self.recording_config = Some(config);

        print_line(&format!(
            "[RECORDING] Enabled with custom config for session '{}'",
            session_name
        ));
        self
    }

    /// Add a replay node from a recording file.
    ///
    /// The replay node will output exactly what was recorded, allowing
    /// mix-and-match debugging with live nodes.
    ///
    /// # Example
    /// ```ignore
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::new();
    /// scheduler.add(live_sensor).order(0).done();  // Live node
    /// scheduler.add_replay(
    ///     PathBuf::from("~/.horus/recordings/crash/motor_node@abc123.horus"),
    ///     1,  // priority
    /// ).expect("Failed to load recording");
    /// ```
    pub fn add_replay(&mut self, recording_path: PathBuf, priority: u32) -> HorusResult<&mut Self> {
        let replayer = NodeReplayer::load(&recording_path)
            .map_err(|e| horus_internal!("Failed to load recording: {}", e))?;

        let node_name = replayer.recording().node_name.clone();
        let node_id = replayer.recording().node_id.clone();

        print_line(&format!(
            "[REPLAY] Loading '{}' from recording (ticks {}-{})",
            node_name,
            replayer.recording().first_tick,
            replayer.recording().last_tick
        ));

        let replay_node = ReplayNode::new(node_name.clone(), node_id.clone());
        self.replay_nodes.insert(node_name.clone(), replayer);

        let replay_tier = NodeTier::default();
        self.nodes.push(RegisteredNode {
            node: Box::new(replay_node),
            priority,
            initialized: false,
            context: None,
            rate_hz: None,
            last_tick: None,
            failure_handler: super::super::fault_tolerance::FailureHandler::new(
                replay_tier.default_failure_policy(),
            ),
            is_rt_node: false,
            wcet_budget: None,
            deadline: None,
            recorder: None,
            is_stopped: false,
            is_paused: false,
        });

        self.nodes.sort_by_key(|n| n.priority);
        Ok(self)
    }

    /// Replay an entire scheduler recording.
    ///
    /// All nodes from the recording will be loaded and replayed.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::replay_from(
    ///     PathBuf::from("~/.horus/recordings/crash/scheduler@abc123.horus")
    /// ).expect("Failed to load scheduler recording");
    /// scheduler.run();
    /// ```
    pub fn replay_from(scheduler_path: PathBuf) -> HorusResult<Self> {
        let scheduler_recording = SchedulerRecording::load(&scheduler_path)
            .map_err(|e| horus_internal!("Failed to load scheduler recording: {}", e))?;

        let session_dir = scheduler_path.parent().unwrap_or(&scheduler_path);
        let mut scheduler =
            Self::new().with_name(&format!("Replay({})", scheduler_recording.session_name));

        scheduler.replay_mode = Some(ReplayMode::Full {
            scheduler_path: scheduler_path.clone(),
        });

        print_line(&format!(
            "[REPLAY] Loading scheduler recording with {} nodes, {} ticks",
            scheduler_recording.node_recordings.len(),
            scheduler_recording.total_ticks
        ));

        for (node_id, relative_path) in &scheduler_recording.node_recordings {
            let node_path = session_dir.join(relative_path);
            if node_path.exists() {
                if let Err(e) = scheduler.add_replay(node_path, 0) {
                    print_line(&format!(
                        "Warning: Failed to load node '{}': {}",
                        node_id, e
                    ));
                }
            }
        }

        Ok(scheduler)
    }

    /// Set replay to start at a specific tick (time travel).
    pub fn start_at_tick(mut self, tick: u64) -> Self {
        self.current_tick = tick;
        for replayer in self.replay_nodes.values_mut() {
            replayer.seek(tick);
        }
        print_line(&format!("[REPLAY] Starting at tick {}", tick));
        self
    }

    /// Set an override value for what-if testing during replay.
    pub fn with_override(mut self, node_name: &str, output_name: &str, value: Vec<u8>) -> Self {
        self.replay_overrides
            .entry(node_name.to_string())
            .or_default()
            .insert(output_name.to_string(), value);
        print_line(&format!(
            "[REPLAY] Override set: {}.{}",
            node_name, output_name
        ));
        self
    }

    /// Set replay to stop at a specific tick.
    pub fn stop_at_tick(mut self, tick: u64) -> Self {
        self.replay_stop_tick = Some(tick);
        print_line(&format!("[REPLAY] Will stop at tick {}", tick));
        self
    }

    /// Set replay speed multiplier.
    pub fn with_replay_speed(mut self, speed: f64) -> Self {
        self.replay_speed = speed.clamp(0.01, 100.0);
        self
    }

    /// Check if recording is enabled.
    pub fn is_recording(&self) -> bool {
        self.recording_config.is_some()
    }

    /// Check if in replay mode.
    pub fn is_replaying(&self) -> bool {
        self.replay_mode.is_some()
    }

    /// Get the current tick number.
    pub fn current_tick(&self) -> u64 {
        self.current_tick
    }

    /// Stop recording and save all data to disk.
    ///
    /// Call this before shutting down to ensure recordings are saved.
    pub fn stop_recording(&mut self) -> HorusResult<Vec<PathBuf>> {
        let mut saved_paths = Vec::new();

        if let Some(ref config) = self.recording_config {
            for registered in self.nodes.iter_mut() {
                if let Some(ref mut recorder) = registered.recorder {
                    match recorder.finish() {
                        Ok(path) => {
                            print_line(&format!("[RECORDING] Saved: {}", path.display()));
                            saved_paths.push(path);
                        }
                        Err(e) => {
                            print_line(&format!(
                                "Failed to save recording for '{}': {}",
                                registered.node.name(),
                                e
                            ));
                        }
                    }
                }
            }

            if let Some(ref mut scheduler_rec) = self.scheduler_recording {
                scheduler_rec.finish();
                let path = config.scheduler_path(&scheduler_rec.scheduler_id);
                if let Err(e) = scheduler_rec.save(&path) {
                    print_line(&format!("Failed to save scheduler recording: {}", e));
                } else {
                    print_line(&format!("[RECORDING] Saved scheduler: {}", path.display()));
                    saved_paths.push(path);
                }
            }
        }

        self.recording_config = None;
        Ok(saved_paths)
    }

    /// List all available recording sessions.
    pub fn list_recordings() -> HorusResult<Vec<String>> {
        let manager = RecordingManager::new();
        manager
            .list_sessions()
            .map_err(|e| horus_internal!("Failed to list recordings: {}", e))
    }

    /// Delete a recording session.
    pub fn delete_recording(session_name: &str) -> HorusResult<()> {
        let manager = RecordingManager::new();
        manager
            .delete_session(session_name)
            .map_err(|e| horus_internal!("Failed to delete recording: {}", e))
    }
}

/// Generate a unique scheduler ID from timestamp and process ID.
fn generate_scheduler_id() -> String {
    format!(
        "{:x}{:x}",
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_nanos() as u64)
            .unwrap_or(0),
        std::process::id() as u64
    )
}
