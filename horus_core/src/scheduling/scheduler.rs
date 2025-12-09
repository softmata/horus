use crate::core::{Node, NodeHeartbeat, NodeInfo};
use crate::error::HorusResult;
use crate::memory::platform::shm_heartbeats_dir;
use colored::Colorize;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

// Record/Replay imports
use super::record_replay::{
    NodeRecorder, NodeReplayer, RecordingConfig, RecordingManager, ReplayMode, ReplayNode,
    SchedulerRecording,
};

// Global flag for SIGTERM handling
static SIGTERM_RECEIVED: AtomicBool = AtomicBool::new(false);

/// SIGTERM signal handler - cleans up session and exits
///
/// # Safety
/// This is a signal handler and must only call async-signal-safe functions.
/// We set a flag and let the main loop do the actual cleanup.
#[cfg(unix)]
extern "C" fn sigterm_handler(_signum: libc::c_int) {
    SIGTERM_RECEIVED.store(true, Ordering::SeqCst);
}

// Import intelligence modules
use super::executors::{AsyncIOExecutor, AsyncResult, BackgroundExecutor, ParallelExecutor};
use super::fault_tolerance::CircuitBreaker;
use super::intelligence::{DependencyGraph, ExecutionTier, RuntimeProfiler, TierClassifier};
use super::jit::CompiledDataflow;
use super::safety_monitor::SafetyMonitor;
use tokio::sync::mpsc;

/// Enhanced node registration info with lifecycle tracking and per-node rate control
struct RegisteredNode {
    node: Box<dyn Node>,
    priority: u32,
    logging_enabled: bool,
    initialized: bool,
    context: Option<NodeInfo>,
    rate_hz: Option<f64>, // Per-node rate control (None = use global scheduler rate)
    last_tick: Option<Instant>, // Last tick time for rate limiting
    circuit_breaker: CircuitBreaker, // Fault tolerance
    is_rt_node: bool,     // Track if this is a real-time node
    wcet_budget: Option<Duration>, // WCET budget for RT nodes
    deadline: Option<Duration>, // Deadline for RT nodes
    is_jit_compiled: bool, // Track if node uses JIT compilation
    jit_stats: Option<CompiledDataflow>, // JIT compilation statistics
    // Record/Replay support
    recorder: Option<NodeRecorder>, // Active recording (None if not recording)
    #[allow(dead_code)] // Stored for future replay-aware scheduling
    is_replay_node: bool, // True if this node is replaying recorded data
}

/// Central orchestrator: holds nodes, drives the tick loop.
pub struct Scheduler {
    nodes: Vec<RegisteredNode>,
    running: Arc<Mutex<bool>>,
    last_instant: Instant,
    last_snapshot: Instant,
    scheduler_name: String,
    working_dir: PathBuf,

    // Intelligence layer (internal, not exposed via API)
    profiler: RuntimeProfiler,
    dependency_graph: Option<DependencyGraph>,
    classifier: Option<TierClassifier>,
    parallel_executor: ParallelExecutor,
    async_io_executor: Option<AsyncIOExecutor>,
    async_result_rx: Option<mpsc::UnboundedReceiver<AsyncResult>>,
    async_result_tx: Option<mpsc::UnboundedSender<AsyncResult>>,
    background_executor: Option<BackgroundExecutor>,
    learning_complete: bool,

    // JIT compilation for ultra-fast nodes
    jit_compiled_nodes: HashMap<String, CompiledDataflow>,

    // Configuration (stored for runtime use)
    config: Option<super::config::SchedulerConfig>,

    // Safety monitor for real-time critical systems
    safety_monitor: Option<SafetyMonitor>,

    // === New runtime features ===
    // Tick rate enforcement
    tick_period: Duration,

    // Checkpoint system
    checkpoint_manager: Option<super::checkpoint::CheckpointManager>,

    // Black box flight recorder
    blackbox: Option<super::blackbox::BlackBox>,

    // Telemetry
    telemetry: Option<super::telemetry::TelemetryManager>,

    // Redundancy manager
    redundancy: Option<super::redundancy::RedundancyManager>,

    // === Deterministic topology tracking ===
    // Whether topology is locked (no more nodes can be added)
    #[allow(dead_code)] // Reserved for future deterministic mode
    topology_locked: bool,

    // Collected topology from all nodes (for validation)
    #[allow(dead_code)] // Reserved for future deterministic mode
    collected_publishers: Vec<(String, String, String)>, // (node_name, topic, type)
    #[allow(dead_code)] // Reserved for future deterministic mode
    collected_subscribers: Vec<(String, String, String)>, // (node_name, topic, type)

    // === Record/Replay System ===
    // Recording configuration (None = recording disabled)
    recording_config: Option<RecordingConfig>,
    // Scheduler-level recording (tracks all nodes)
    scheduler_recording: Option<SchedulerRecording>,
    // Replay mode (None = live execution)
    replay_mode: Option<ReplayMode>,
    // Replay nodes loaded from recordings
    replay_nodes: HashMap<String, NodeReplayer>,
    // Value overrides for what-if testing during replay
    replay_overrides: HashMap<String, HashMap<String, Vec<u8>>>,
    // Current tick number for recording/replay
    current_tick: u64,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler {
    /// Create an empty scheduler with **deterministic defaults**.
    ///
    /// By default, the scheduler:
    /// - Disables learning phase (no runtime profiling)
    /// - Uses sequential execution (predictable order)
    /// - Is fully deterministic from tick 0
    ///
    /// For adaptive optimization, use `Scheduler::new().enable_learning()`
    /// or load a pre-computed profile with `Scheduler::with_profile()`.
    pub fn new() -> Self {
        let running = Arc::new(Mutex::new(true));
        let now = Instant::now();

        Self {
            nodes: Vec::new(),
            running,
            last_instant: now,
            last_snapshot: now,
            scheduler_name: "DefaultScheduler".to_string(),
            working_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),

            // Initialize intelligence layer - DETERMINISTIC BY DEFAULT
            profiler: RuntimeProfiler::new_default(),
            dependency_graph: None,
            classifier: None,
            parallel_executor: ParallelExecutor::new(),
            async_io_executor: None,
            async_result_rx: None,
            async_result_tx: None,
            background_executor: None,
            learning_complete: true, // CHANGED: Default to deterministic (no learning)

            // JIT compilation
            jit_compiled_nodes: HashMap::new(),

            // Configuration
            config: None,

            // Safety monitor
            safety_monitor: None,

            // New runtime features (disabled by default)
            tick_period: Duration::from_micros(16667), // ~60Hz default
            checkpoint_manager: None,
            blackbox: None,
            telemetry: None,
            redundancy: None,

            // Deterministic topology tracking
            topology_locked: false,
            collected_publishers: Vec::new(),
            collected_subscribers: Vec::new(),

            // Record/Replay system (disabled by default)
            recording_config: None,
            scheduler_recording: None,
            replay_mode: None,
            replay_nodes: HashMap::new(),
            replay_overrides: HashMap::new(),
            current_tick: 0,
        }
    }

    /// Apply a configuration preset to this scheduler (builder pattern)
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::SchedulerConfig;
    /// let mut scheduler = Scheduler::new()
    ///     .with_config(SchedulerConfig::hard_realtime())
    ///     .disable_learning();
    /// ```
    #[allow(deprecated)] // set_config is intentionally used here for builder pattern
    pub fn with_config(mut self, config: super::config::SchedulerConfig) -> Self {
        self.set_config(config);
        self
    }

    /// Pre-allocate node capacity (prevents reallocations during runtime)
    ///
    /// Call this before adding nodes for deterministic memory behavior.
    pub fn with_capacity(mut self, capacity: usize) -> Self {
        self.nodes.reserve(capacity);
        self
    }

    /// Enable deterministic execution for reproducible, bit-exact behavior
    ///
    /// When enabled:
    /// - Learning disabled (no adaptive optimizations)
    /// - Deterministic collections (sorted iteration order)
    /// - Logical clock support (opt-in via config)
    /// - Predictable memory allocation (opt-in via config)
    ///
    /// Performance impact: ~5-10% slower (optimized deterministic implementation)
    ///
    /// Use cases:
    /// - Simulation (Gazebo, Unity integration)
    /// - Testing (reproducible tests in CI/CD)
    /// - Debugging (replay exact behavior)
    /// - Certification (FDA/CE requirements)
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// let scheduler = Scheduler::new()
    ///     .enable_determinism();  // Reproducible execution
    /// ```
    pub fn enable_determinism(self) -> Self {
        // Disable learning for deterministic behavior
        self.disable_learning().with_name("DeterministicScheduler")
    }

    /// Disable the learning phase for predictable startup behavior
    ///
    /// When disabled:
    /// - No ~100-tick profiling phase at startup
    /// - No automatic tier classification of nodes
    /// - No auto-JIT compilation of ultra-fast nodes after learning
    /// - Nodes that declare `supports_jit()` still get compiled at add-time
    ///
    /// Use cases:
    /// - Real-time systems that need immediate predictable execution
    /// - Testing/debugging where profiling overhead is unwanted
    /// - Short-lived schedulers where learning would never complete
    ///
    /// For full deterministic behavior (reproducible execution), use
    /// `enable_determinism()` instead which also disables learning.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// let scheduler = Scheduler::new()
    ///     .disable_learning();  // Skip profiling, run immediately
    /// ```
    pub fn disable_learning(mut self) -> Self {
        self.learning_complete = true;
        self.classifier = None;
        self
    }

    /// Enable safety monitor with maximum allowed deadline misses
    pub fn with_safety_monitor(mut self, max_deadline_misses: u64) -> Self {
        self.safety_monitor = Some(SafetyMonitor::new(max_deadline_misses));
        self
    }

    /// Set scheduler name (for debugging/logging)
    pub fn with_name(mut self, name: &str) -> Self {
        self.scheduler_name = name.to_string();
        self
    }

    // ============================================================================
    // Record/Replay System
    // ============================================================================

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
        // Generate unique scheduler ID from timestamp and process ID
        let scheduler_id = format!(
            "{:x}{:x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            std::process::id() as u64
        );

        self.scheduler_recording = Some(SchedulerRecording::new(&scheduler_id, session_name));
        self.recording_config = Some(config);

        println!(
            "{}",
            format!(
                "[RECORDING] Enabled for session '{}' (scheduler@{})",
                session_name, scheduler_id
            )
            .green()
        );
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
        // Generate unique scheduler ID from timestamp and process ID
        let scheduler_id = format!(
            "{:x}{:x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            std::process::id() as u64
        );
        let session_name = config.session_name.clone();

        self.scheduler_recording = Some(SchedulerRecording::new(&scheduler_id, &session_name));
        self.recording_config = Some(config);

        println!(
            "{}",
            format!(
                "[RECORDING] Enabled with custom config for session '{}'",
                session_name
            )
            .green()
        );
        self
    }

    /// Add a replay node from a recording file.
    ///
    /// The replay node will output exactly what was recorded, allowing
    /// mix-and-match debugging with live nodes.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::new();
    /// scheduler.add(Box::new(live_sensor), 0, None);  // Live node
    /// scheduler.add_replay(
    ///     PathBuf::from("~/.horus/recordings/crash/motor_node@abc123.horus"),
    ///     1,  // priority
    /// ).expect("Failed to load recording");
    /// ```
    pub fn add_replay(&mut self, recording_path: PathBuf, priority: u32) -> HorusResult<&mut Self> {
        let replayer = NodeReplayer::load(&recording_path).map_err(|e| {
            crate::error::HorusError::Internal(format!("Failed to load recording: {}", e))
        })?;

        let node_name = replayer.recording().node_name.clone();
        let node_id = replayer.recording().node_id.clone();

        println!(
            "{}",
            format!(
                "[REPLAY] Loading '{}' from recording (ticks {}-{})",
                node_name,
                replayer.recording().first_tick,
                replayer.recording().last_tick
            )
            .cyan()
        );

        // Create a ReplayNode wrapper
        let replay_node = ReplayNode::new(node_name.clone(), node_id.clone());

        // Store the replayer
        self.replay_nodes.insert(node_name.clone(), replayer);

        // Add as a registered node with replay flag
        self.nodes.push(RegisteredNode {
            node: Box::new(replay_node),
            priority,
            logging_enabled: true,
            initialized: false,
            context: None,
            rate_hz: None,
            last_tick: None,
            circuit_breaker: CircuitBreaker::new(5, 3, 30000), // 5 failures, 3 success, 30s timeout
            is_rt_node: false,
            wcet_budget: None,
            deadline: None,
            is_jit_compiled: false,
            jit_stats: None,
            recorder: None,
            is_replay_node: true,
        });

        // Sort nodes by priority
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
        let scheduler_recording = SchedulerRecording::load(&scheduler_path).map_err(|e| {
            crate::error::HorusError::Internal(format!("Failed to load scheduler recording: {}", e))
        })?;

        let session_dir = scheduler_path.parent().unwrap_or(&scheduler_path);
        let mut scheduler =
            Self::new().with_name(&format!("Replay({})", scheduler_recording.session_name));

        scheduler.replay_mode = Some(ReplayMode::Full {
            scheduler_path: scheduler_path.clone(),
        });

        println!(
            "{}",
            format!(
                "[REPLAY] Loading scheduler recording with {} nodes, {} ticks",
                scheduler_recording.node_recordings.len(),
                scheduler_recording.total_ticks
            )
            .cyan()
        );

        // Load all node recordings
        for (node_id, relative_path) in &scheduler_recording.node_recordings {
            let node_path = session_dir.join(relative_path);
            if node_path.exists() {
                if let Err(e) = scheduler.add_replay(node_path, 0) {
                    eprintln!("Warning: Failed to load node '{}': {}", node_id, e);
                }
            }
        }

        Ok(scheduler)
    }

    /// Set replay to start at a specific tick (time travel).
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::replay_from(
    ///     PathBuf::from("~/.horus/recordings/crash/scheduler@abc123.horus")
    /// ).expect("Failed to load")
    ///     .start_at_tick(1500);  // Jump to tick 1500
    /// ```
    pub fn start_at_tick(mut self, tick: u64) -> Self {
        self.current_tick = tick;

        // Seek all replayers to this tick
        for replayer in self.replay_nodes.values_mut() {
            replayer.seek(tick);
        }

        println!("{}", format!("[REPLAY] Starting at tick {}", tick).cyan());
        self
    }

    /// Set an override value for what-if testing during replay.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::replay_from(
    ///     PathBuf::from("~/.horus/recordings/crash/scheduler@abc123.horus")
    /// ).expect("Failed to load")
    ///     .with_override("sensor_node", "temperature", vec![0, 0, 200, 65]); // Override temp=25.0
    /// ```
    pub fn with_override(mut self, node_name: &str, output_name: &str, value: Vec<u8>) -> Self {
        self.replay_overrides
            .entry(node_name.to_string())
            .or_default()
            .insert(output_name.to_string(), value);

        println!(
            "{}",
            format!("[REPLAY] Override set: {}.{}", node_name, output_name).yellow()
        );
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
            // Save all node recordings
            for registered in self.nodes.iter_mut() {
                if let Some(ref mut recorder) = registered.recorder {
                    match recorder.finish() {
                        Ok(path) => {
                            println!(
                                "{}",
                                format!("[RECORDING] Saved: {}", path.display()).green()
                            );
                            saved_paths.push(path);
                        }
                        Err(e) => {
                            eprintln!(
                                "Failed to save recording for '{}': {}",
                                registered.node.name(),
                                e
                            );
                        }
                    }
                }
            }

            // Save scheduler recording
            if let Some(ref mut scheduler_rec) = self.scheduler_recording {
                scheduler_rec.finish();
                let path = config.scheduler_path(&scheduler_rec.scheduler_id);
                if let Err(e) = scheduler_rec.save(&path) {
                    eprintln!("Failed to save scheduler recording: {}", e);
                } else {
                    println!(
                        "{}",
                        format!("[RECORDING] Saved scheduler: {}", path.display()).green()
                    );
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
        manager.list_sessions().map_err(|e| {
            crate::error::HorusError::Internal(format!("Failed to list recordings: {}", e))
        })
    }

    /// Delete a recording session.
    pub fn delete_recording(session_name: &str) -> HorusResult<()> {
        let manager = RecordingManager::new();
        manager.delete_session(session_name).map_err(|e| {
            crate::error::HorusError::Internal(format!("Failed to delete recording: {}", e))
        })
    }

    // ============================================================================
    // Deterministic Topology Validation (copper-rs level guarantees)
    // ============================================================================

    /// Validate the system topology before running.
    ///
    /// This checks that:
    /// - All topics have at least one publisher and one subscriber
    /// - Type names match between publishers and subscribers
    /// - No orphaned topics exist
    ///
    /// Returns a list of topology errors if validation fails.
    ///
    /// # Example
    /// ```ignore
    /// let errors = scheduler.validate_topology();
    /// if !errors.is_empty() {
    ///     for err in &errors {
    ///         eprintln!("Topology error: {}", err);
    ///     }
    ///     panic!("Topology validation failed");
    /// }
    /// ```
    pub fn validate_topology(&self) -> Vec<String> {
        let mut errors = Vec::new();

        // Check if deterministic mode requires complete connections
        let require_complete = self
            .config
            .as_ref()
            .and_then(|c| c.deterministic.as_ref())
            .map(|d| d.require_complete_connections)
            .unwrap_or(false);

        // Build topic maps
        let mut publisher_topics: std::collections::HashMap<String, Vec<(String, String)>> =
            std::collections::HashMap::new();
        let mut subscriber_topics: std::collections::HashMap<String, Vec<(String, String)>> =
            std::collections::HashMap::new();

        for (node_name, topic, type_name) in &self.collected_publishers {
            publisher_topics
                .entry(topic.clone())
                .or_default()
                .push((node_name.clone(), type_name.clone()));
        }

        for (node_name, topic, type_name) in &self.collected_subscribers {
            subscriber_topics
                .entry(topic.clone())
                .or_default()
                .push((node_name.clone(), type_name.clone()));
        }

        // Check for orphaned publishers (no subscribers)
        if require_complete {
            for (topic, publishers) in &publisher_topics {
                if !subscriber_topics.contains_key(topic) {
                    errors.push(format!(
                        "Topic '{}' has publishers {:?} but no subscribers",
                        topic,
                        publishers.iter().map(|(n, _)| n).collect::<Vec<_>>()
                    ));
                }
            }

            // Check for orphaned subscribers (no publishers)
            for (topic, subscribers) in &subscriber_topics {
                if !publisher_topics.contains_key(topic) {
                    errors.push(format!(
                        "Topic '{}' has subscribers {:?} but no publishers",
                        topic,
                        subscribers.iter().map(|(n, _)| n).collect::<Vec<_>>()
                    ));
                }
            }
        }

        // Check for type mismatches
        for (topic, subscribers) in &subscriber_topics {
            if let Some(publishers) = publisher_topics.get(topic) {
                for (sub_node, sub_type) in subscribers {
                    for (pub_node, pub_type) in publishers {
                        if sub_type != pub_type {
                            errors.push(format!(
                                "Type mismatch on topic '{}': publisher '{}' sends '{}', subscriber '{}' expects '{}'",
                                topic, pub_node, pub_type, sub_node, sub_type
                            ));
                        }
                    }
                }
            }
        }

        errors
    }

    /// Lock the topology, preventing further node additions.
    ///
    /// After calling this, any attempt to add nodes will panic in strict
    /// deterministic mode. This ensures the execution plan is fixed.
    ///
    /// # Example
    /// ```ignore
    /// scheduler.add(Box::new(node1), 0, None);
    /// scheduler.add(Box::new(node2), 1, None);
    /// scheduler.lock_topology();  // No more nodes can be added
    /// scheduler.run();
    /// ```
    pub fn lock_topology(&mut self) -> &mut Self {
        self.topology_locked = true;
        println!(
            "[DETERMINISTIC] Topology locked with {} publishers, {} subscribers",
            self.collected_publishers.len(),
            self.collected_subscribers.len()
        );
        self
    }

    /// Get the collected topology for inspection.
    ///
    /// Returns (publishers, subscribers) where each is a list of
    /// (node_name, topic_name, type_name) tuples.
    #[allow(clippy::type_complexity)]
    pub fn get_topology(&self) -> (&[(String, String, String)], &[(String, String, String)]) {
        (&self.collected_publishers, &self.collected_subscribers)
    }

    /// Check if topology is currently locked.
    pub fn is_topology_locked(&self) -> bool {
        self.topology_locked
    }

    // ============================================================================
    // Profile-Based Optimization (Deterministic Alternative to Learning)
    // ============================================================================

    /// Load a pre-computed profile for deterministic, optimized execution.
    ///
    /// This is the recommended way to get both determinism AND optimization:
    /// 1. Profile once: `horus profile my_robot.rs --output robot.profile`
    /// 2. Use profile: `Scheduler::with_profile("robot.profile")`
    ///
    /// The scheduler will use the pre-computed tier classifications without
    /// any runtime learning phase, ensuring deterministic execution from tick 0.
    ///
    /// # Example
    /// ```ignore
    /// use horus_core::Scheduler;
    ///
    /// let scheduler = Scheduler::with_profile("robot.profile")?;
    /// // Deterministic AND optimized - best of both worlds
    /// ```
    pub fn with_profile<P: AsRef<std::path::Path>>(
        path: P,
    ) -> Result<Self, super::intelligence::ProfileError> {
        let profile = super::intelligence::ProfileData::load(&path)?;

        // Print warnings if hardware differs
        let warnings = profile.check_compatibility();
        for warning in &warnings {
            eprintln!("[PROFILE] Warning: {}", warning);
        }

        let mut scheduler = Self::new();
        scheduler.scheduler_name = format!("ProfiledScheduler({})", profile.name);

        // Store profile for use when adding nodes
        // The classifier will be populated from the profile
        let mut classifier = super::intelligence::TierClassifier {
            assignments: std::collections::HashMap::new(),
        };

        for (name, node_profile) in &profile.nodes {
            classifier
                .assignments
                .insert(name.clone(), node_profile.tier.to_execution_tier());
        }

        scheduler.classifier = Some(classifier);

        println!(
            "[OK] Loaded profile '{}' ({} nodes)",
            profile.name,
            profile.nodes.len()
        );
        println!("   - Determinism: ENABLED (from profile)");
        println!("   - Execution: Optimized per-node tiers");

        Ok(scheduler)
    }

    /// Enable the learning phase (opt-in for adaptive optimization).
    ///
    /// **Warning**: This makes execution non-deterministic!
    ///
    /// The learning phase profiles nodes for ~100 ticks and then
    /// automatically classifies them into execution tiers. Results
    /// may vary between runs due to system noise.
    ///
    /// For deterministic optimization, use `with_profile()` instead.
    ///
    /// # Example
    /// ```ignore
    /// use horus_core::Scheduler;
    ///
    /// // Opt-in to non-deterministic learning
    /// let scheduler = Scheduler::new()
    ///     .enable_learning();  // WARNING: Non-deterministic!
    /// ```
    pub fn enable_learning(mut self) -> Self {
        self.learning_complete = false;
        self.classifier = None;
        println!("[WARN] Learning phase enabled - execution will be non-deterministic");
        println!("       For deterministic optimization, use Scheduler::with_profile() instead");
        self
    }

    /// Add a node with explicit tier annotation (deterministic optimization).
    ///
    /// Use this when you know a node's characteristics at compile time.
    /// This avoids the non-deterministic runtime learning phase.
    ///
    /// # Arguments
    /// * `node` - The node to add
    /// * `priority` - Priority level (0 = highest)
    /// * `tier` - Explicit execution tier
    ///
    /// # Example
    /// ```ignore
    /// use horus_core::{Scheduler, scheduling::NodeTier};
    ///
    /// let scheduler = Scheduler::new()
    ///     .add_with_tier(Box::new(pid_controller), 0, NodeTier::Jit)
    ///     .add_with_tier(Box::new(sensor_reader), 1, NodeTier::Fast)
    ///     .add_with_tier(Box::new(data_logger), 5, NodeTier::Background);
    /// ```
    pub fn add_with_tier(
        &mut self,
        node: Box<dyn Node>,
        priority: u32,
        tier: super::intelligence::NodeTier,
    ) -> &mut Self {
        let node_name = node.name().to_string();

        // Add node with default logging
        self.add(node, priority, None);

        // Set explicit tier in classifier
        if self.classifier.is_none() {
            self.classifier = Some(super::intelligence::TierClassifier {
                assignments: std::collections::HashMap::new(),
            });
        }

        if let Some(ref mut classifier) = self.classifier {
            classifier
                .assignments
                .insert(node_name.clone(), tier.to_execution_tier());
        }

        println!("Added node '{}' with explicit tier: {:?}", node_name, tier);

        self
    }

    // ============================================================================
    // Convenience Constructors (thin wrappers for common patterns)
    // ============================================================================

    /// Create a hard real-time scheduler (convenience constructor)
    ///
    /// This is equivalent to:
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::SchedulerConfig;
    /// Scheduler::new()
    ///     .with_config(SchedulerConfig::hard_realtime())
    ///     .with_capacity(128)
    ///     .enable_determinism()
    ///     .with_safety_monitor(3);
    /// ```
    ///
    /// After construction, call OS integration methods:
    /// - `set_realtime_priority(99)` - SCHED_FIFO scheduling
    /// - `pin_to_cpu(7)` - Pin to isolated core
    /// - `lock_memory()` - Prevent page faults
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// fn main() -> Result<(), Box<dyn std::error::Error>> {
    ///     let mut scheduler = Scheduler::new_realtime()?;
    ///     scheduler.set_realtime_priority(99)?;
    ///     scheduler.pin_to_cpu(7)?;
    ///     scheduler.lock_memory()?;
    ///     Ok(())
    /// }
    /// ```
    pub fn new_realtime() -> crate::error::HorusResult<Self> {
        let sched = Self::new()
            .with_config(super::config::SchedulerConfig::hard_realtime())
            .with_capacity(128)
            .enable_determinism() // Use unified determinism API
            .with_safety_monitor(3)
            .with_name("RealtimeScheduler");

        println!("[FAST] Real-time scheduler initialized");
        println!("   - Config: hard_realtime() preset");
        println!("   - Capacity: 128 nodes pre-allocated");
        println!("   - Determinism: ENABLED");
        println!("   - Safety monitor: ENABLED (max 3 misses)");
        println!("   - Next: Call set_realtime_priority(99), pin_to_cpu(N), lock_memory()");

        Ok(sched)
    }

    /// Create a deterministic scheduler (convenience constructor)
    ///
    /// This is equivalent to:
    /// ```no_run
    /// use horus_core::Scheduler;
    /// Scheduler::new()
    ///     .enable_determinism();
    /// ```
    ///
    /// Provides reproducible, bit-exact execution for simulation and testing.
    pub fn new_deterministic() -> Self {
        let sched = Self::new().enable_determinism();

        println!("[OK] Deterministic scheduler initialized");
        println!("   - Determinism: ENABLED");
        println!("   - Execution: Reproducible, bit-exact");
        println!("   - Use for: Simulation, testing, certification");

        sched
    }

    // ============================================================================
    // OS Integration Methods (low-level, genuinely different from config)
    // ============================================================================

    /// Set real-time priority using SCHED_FIFO (Linux RT-PREEMPT required)
    ///
    /// # Arguments
    /// * `priority` - Priority level (1-99, higher = more important)
    ///   - 99: Critical control loops (motors, safety)
    ///   - 90: High-priority sensors
    ///   - 80: Normal control
    ///   - 50-70: Background tasks
    ///
    /// # Requirements
    /// - RT-PREEMPT kernel (linux-image-rt)
    /// - CAP_SYS_NICE capability or root
    ///
    /// # Example
    /// ```ignore
    /// scheduler.set_realtime_priority(99)?;  // Highest priority
    /// ```
    pub fn set_realtime_priority(&self, priority: i32) -> crate::error::HorusResult<()> {
        if !(1..=99).contains(&priority) {
            return Err(crate::error::HorusError::config(
                "Priority must be between 1 and 99",
            ));
        }

        #[cfg(target_os = "linux")]
        unsafe {
            use libc::{sched_param, sched_setscheduler, SCHED_FIFO};

            let param = sched_param {
                sched_priority: priority,
            };

            if sched_setscheduler(0, SCHED_FIFO, &param) != 0 {
                let err = std::io::Error::last_os_error();
                return Err(crate::error::HorusError::Internal(format!(
                    "Failed to set real-time priority: {}. \
                     Ensure you have RT-PREEMPT kernel and CAP_SYS_NICE capability.",
                    err
                )));
            }

            println!("[OK] Real-time priority set to {} (SCHED_FIFO)", priority);
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "Real-time priority scheduling is only supported on Linux".to_string(),
            ))
        }
    }

    /// Pin scheduler to a specific CPU core (prevent context switches)
    ///
    /// # Arguments
    /// * `cpu_id` - CPU core number (0-N)
    ///
    /// # Best Practices
    /// - Use isolated cores (boot with isolcpus=7 kernel parameter)
    /// - Reserve core for RT tasks only
    /// - Disable hyperthreading for predictable performance
    ///
    /// # Example
    /// ```ignore
    /// // Pin to isolated core 7
    /// scheduler.pin_to_cpu(7)?;
    /// ```
    pub fn pin_to_cpu(&self, cpu_id: usize) -> crate::error::HorusResult<()> {
        #[cfg(target_os = "linux")]
        unsafe {
            use libc::{cpu_set_t, sched_setaffinity, CPU_SET, CPU_ZERO};

            let mut cpuset: cpu_set_t = std::mem::zeroed();
            CPU_ZERO(&mut cpuset);
            CPU_SET(cpu_id, &mut cpuset);

            if sched_setaffinity(0, std::mem::size_of::<cpu_set_t>(), &cpuset) != 0 {
                let err = std::io::Error::last_os_error();
                return Err(crate::error::HorusError::Internal(format!(
                    "Failed to set CPU affinity: {}",
                    err
                )));
            }

            println!("[OK] Scheduler pinned to CPU core {}", cpu_id);
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "CPU pinning is only supported on Linux".to_string(),
            ))
        }
    }

    /// Lock all memory pages to prevent page faults (critical for <20μs latency)
    ///
    /// This prevents the OS from swapping out scheduler memory, which would
    /// cause multi-millisecond delays. Essential for hard real-time systems.
    ///
    /// # Requirements
    /// - Sufficient locked memory limit (ulimit -l)
    /// - CAP_IPC_LOCK capability or root
    ///
    /// # Warning
    /// This locks ALL current and future memory allocations. Ensure your
    /// application has bounded memory usage.
    ///
    /// # Example
    /// ```ignore
    /// scheduler.lock_memory()?;
    /// ```
    pub fn lock_memory(&self) -> crate::error::HorusResult<()> {
        #[cfg(target_os = "linux")]
        unsafe {
            use libc::{mlockall, MCL_CURRENT, MCL_FUTURE};

            if mlockall(MCL_CURRENT | MCL_FUTURE) != 0 {
                let err = std::io::Error::last_os_error();
                return Err(crate::error::HorusError::Internal(format!(
                    "Failed to lock memory: {}. \
                     Check ulimit -l and ensure CAP_IPC_LOCK capability.",
                    err
                )));
            }

            println!("[OK] Memory locked (no page faults)");
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "Memory locking is only supported on Linux".to_string(),
            ))
        }
    }

    /// Pre-fault stack to prevent page faults during execution
    ///
    /// Touches stack pages to ensure they're resident in RAM before
    /// time-critical execution begins.
    ///
    /// # Arguments
    /// * `stack_size` - Stack size to pre-fault (bytes)
    ///
    /// # Example
    /// ```ignore
    /// scheduler.prefault_stack(8 * 1024 * 1024)?;  // 8MB stack
    /// ```
    pub fn prefault_stack(&self, stack_size: usize) -> crate::error::HorusResult<()> {
        // Allocate array on stack and touch each page
        let page_size = 4096; // Standard page size
        let pages = stack_size / page_size;

        // Use volatile writes to prevent optimization
        for i in 0..pages {
            let offset = i * page_size;
            let mut dummy_stack = vec![0u8; page_size];
            unsafe {
                std::ptr::write_volatile(&mut dummy_stack[offset % page_size], 0xFF);
            }
        }

        println!("[OK] Pre-faulted {} KB of stack", stack_size / 1024);
        Ok(())
    }

    /// Add a node with given priority (lower number = higher priority).
    /// If users only use add(node, priority) then logging defaults to false
    /// Automatically detects and wraps RTNode types for real-time support
    ///
    /// # Example
    /// ```ignore
    /// scheduler.add(node, 0, None);  // Highest priority
    /// scheduler.add(node, 10, None); // Medium priority
    /// scheduler.add(node, 100, None); // Low priority
    /// ```
    pub fn add(
        &mut self,
        node: Box<dyn Node>,
        priority: u32,
        logging_enabled: Option<bool>,
    ) -> &mut Self {
        // Check if topology is locked (deterministic mode)
        if self.topology_locked {
            if let Some(ref config) = self.config {
                if let Some(ref det_config) = config.deterministic {
                    if det_config.freeze_topology_after_start {
                        panic!(
                            "Cannot add node '{}': topology is locked in deterministic mode",
                            node.name()
                        );
                    }
                }
            }
        }

        let node_name = node.name().to_string();
        let logging_enabled = logging_enabled.unwrap_or(false);

        // Collect topology from node (for deterministic validation)
        for pub_meta in node.get_publishers() {
            self.collected_publishers.push((
                node_name.clone(),
                pub_meta.topic_name,
                pub_meta.type_name,
            ));
        }
        for sub_meta in node.get_subscribers() {
            self.collected_subscribers.push((
                node_name.clone(),
                sub_meta.topic_name,
                sub_meta.type_name,
            ));
        }

        // Check if this node supports JIT compilation using trait methods
        let is_jit_capable = node.supports_jit();
        let jit_arithmetic_params = node.get_jit_arithmetic_params();
        let jit_compute_fn = node.get_jit_compute();

        // Track the compiled JIT function if available
        let (is_jit_compiled, jit_compiled) = if is_jit_capable {
            // Try to compile if the node provides arithmetic params
            if let Some((factor, offset)) = jit_arithmetic_params {
                match super::jit::JITCompiler::new() {
                    Ok(mut compiler) => {
                        let unique_name = format!("{}_{}", node_name, self.nodes.len());
                        match compiler.compile_arithmetic_node(&unique_name, factor, offset) {
                            Ok(func_ptr) => {
                                println!(
                                    "[JIT] Compiled node '{}' with factor={}, offset={}",
                                    node_name, factor, offset
                                );
                                let compiled = CompiledDataflow {
                                    name: node_name.clone(),
                                    func_ptr,
                                    exec_count: 0,
                                    total_ns: 0,
                                };
                                (true, Some(compiled))
                            }
                            Err(e) => {
                                eprintln!("[JIT] Failed to compile '{}': {}", node_name, e);
                                (false, None)
                            }
                        }
                    }
                    Err(e) => {
                        eprintln!("[JIT] Compiler init failed for '{}': {}", node_name, e);
                        (false, None)
                    }
                }
            } else if jit_compute_fn.is_some() {
                // Node provides a direct compute function
                println!(
                    "[JIT] Node '{}' provides direct compute function",
                    node_name
                );
                (true, None) // Will use get_jit_compute() at runtime
            } else {
                // JIT capable but no compile params - track for stats only
                println!("[JIT] Node '{}' is JIT-capable (tracking stats)", node_name);
                (true, Some(CompiledDataflow::new_stats_only(&node_name)))
            }
        } else {
            (false, None)
        };

        let context = NodeInfo::new(node_name.clone(), logging_enabled);

        // Check if this might be an RT node based on naming patterns or other heuristics
        // In production, you'd want a more robust detection mechanism
        let is_rt_node = node_name.contains("motor")
            || node_name.contains("control")
            || node_name.contains("sensor")
            || node_name.contains("critical");

        // For RT nodes, extract WCET and deadline if available
        // This would normally come from the RTNode trait methods
        let (wcet_budget, deadline) = if is_rt_node {
            // Default RT constraints for demonstration
            (
                Some(Duration::from_micros(100)), // 100μs WCET
                Some(Duration::from_millis(1)),   // 1ms deadline
            )
        } else {
            (None, None)
        };

        // Store JIT compiled function in the global map for fast lookup
        if let Some(ref compiled) = jit_compiled {
            self.jit_compiled_nodes.insert(
                node_name.clone(),
                CompiledDataflow {
                    name: compiled.name.clone(),
                    func_ptr: compiled.func_ptr,
                    exec_count: 0,
                    total_ns: 0,
                },
            );
        }

        // Create node recorder if recording is enabled
        let recorder = if let Some(ref config) = self.recording_config {
            // Generate unique node ID from timestamp and node index
            let node_id = format!(
                "{:x}{:x}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(0),
                self.nodes.len() as u64
            );
            let recorder = NodeRecorder::new(&node_name, &node_id, config.clone());

            // Register with scheduler recording
            if let Some(ref mut scheduler_rec) = self.scheduler_recording {
                let relative_path = format!("{}@{}.horus", node_name, node_id);
                scheduler_rec.add_node_recording(&node_id, &relative_path);
            }

            Some(recorder)
        } else {
            None
        };

        self.nodes.push(RegisteredNode {
            node,
            priority,
            logging_enabled,
            initialized: false,
            context: Some(context),
            rate_hz: None,   // Use global scheduler rate by default
            last_tick: None, // Will be set on first tick
            circuit_breaker: CircuitBreaker::new(5, 3, 5000), // 5 failures to open, 3 successes to close, 5s timeout
            is_rt_node,
            wcet_budget,
            deadline,
            is_jit_compiled,
            jit_stats: jit_compiled, // JIT-compiled dataflow (if available)
            recorder,                // Node recorder (if recording enabled)
            is_replay_node: false,   // Live node, not replay
        });

        println!(
            "Added {} '{}' with priority {} (logging: {})",
            if is_rt_node { "RT node" } else { "node" },
            node_name,
            priority,
            logging_enabled
        );

        self
    }

    /// Add a real-time node with explicit RT constraints
    ///
    /// This method allows precise configuration of RT nodes with WCET budgets,
    /// deadlines, and other real-time constraints.
    ///
    /// # Example
    /// ```ignore
    /// scheduler.add_rt(
    ///     Box::new(MotorControlNode::new("motor")),
    ///     0,  // Highest priority
    ///     Duration::from_micros(100),  // 100μs WCET budget
    ///     Duration::from_millis(1),    // 1ms deadline
    /// );
    /// ```
    pub fn add_rt(
        &mut self,
        node: Box<dyn Node>,
        priority: u32,
        wcet_budget: Duration,
        deadline: Duration,
    ) -> &mut Self {
        let node_name = node.name().to_string();
        let logging_enabled = false; // RT nodes typically don't need logging overhead

        let context = NodeInfo::new(node_name.clone(), logging_enabled);

        self.nodes.push(RegisteredNode {
            node,
            priority,
            logging_enabled,
            initialized: false,
            context: Some(context),
            rate_hz: None,
            last_tick: None,
            circuit_breaker: CircuitBreaker::new(5, 3, 5000),
            is_rt_node: true,
            wcet_budget: Some(wcet_budget),
            deadline: Some(deadline),
            is_jit_compiled: false, // RT nodes typically don't use JIT
            jit_stats: None,
            recorder: None,
            is_replay_node: false,
        });

        println!(
            "Added RT node '{}' with priority {} (WCET: {:?}, deadline: {:?})",
            node_name, priority, wcet_budget, deadline
        );

        // If safety monitor exists, configure it for this node
        if let Some(ref mut monitor) = self.safety_monitor {
            monitor.set_wcet_budget(node_name.clone(), wcet_budget);
            if let Some(ref config) = self.config {
                if config.realtime.watchdog_enabled {
                    let watchdog_timeout =
                        Duration::from_millis(config.realtime.watchdog_timeout_ms);
                    monitor.add_critical_node(node_name, watchdog_timeout);
                }
            }
        }

        self
    }

    /// Set the scheduler name (chainable)
    pub fn name(mut self, name: &str) -> Self {
        self.scheduler_name = name.to_string();
        self
    }

    /// Tick specific nodes by name (runs continuously with the specified nodes)
    pub fn tick(&mut self, node_names: &[&str]) -> HorusResult<()> {
        // Use the same pattern as run() but with node filtering
        self.run_with_filter(Some(node_names), None)
    }

    /// Check if the scheduler is running
    pub fn is_running(&self) -> bool {
        if let Ok(running) = self.running.lock() {
            *running
        } else {
            false
        }
    }

    /// Stop the scheduler
    pub fn stop(&self) {
        if let Ok(mut running) = self.running.lock() {
            *running = false;
        }
    }

    /// Set per-node rate control (chainable)
    ///
    /// Allows individual nodes to run at different frequencies independent of the global scheduler rate.
    /// If a node's rate is not set, it will tick at the global scheduler frequency.
    ///
    /// # Arguments
    /// * `name` - The name of the node
    /// * `rate_hz` - The desired rate in Hz (ticks per second)
    ///
    /// # Example
    /// ```ignore
    /// scheduler.add(sensor, 0, Some(true))
    ///     .set_node_rate("sensor", 100.0);  // Run sensor at 100Hz
    /// ```
    pub fn set_node_rate(&mut self, name: &str, rate_hz: f64) -> &mut Self {
        for registered in self.nodes.iter_mut() {
            if registered.node.name() == name {
                registered.rate_hz = Some(rate_hz);
                registered.last_tick = Some(Instant::now());
                println!("Set node '{}' rate to {:.1} Hz", name, rate_hz);
                break;
            }
        }
        self
    }

    /// Main loop with automatic signal handling and cleanup
    pub fn run(&mut self) -> HorusResult<()> {
        self.run_with_filter(None, None)
    }

    /// Run all nodes for a specified duration, then shutdown gracefully
    pub fn run_for(&mut self, duration: Duration) -> HorusResult<()> {
        self.run_with_filter(None, Some(duration))
    }

    /// Run specific nodes for a specified duration, then shutdown gracefully
    pub fn tick_for(&mut self, node_names: &[&str], duration: Duration) -> HorusResult<()> {
        self.run_with_filter(Some(node_names), Some(duration))
    }

    /// Internal method to run scheduler with optional node filtering and duration
    fn run_with_filter(
        &mut self,
        node_filter: Option<&[&str]>,
        duration: Option<Duration>,
    ) -> HorusResult<()> {
        // Create tokio runtime for nodes that need async
        let rt = tokio::runtime::Runtime::new().map_err(|e| {
            crate::error::HorusError::Internal(format!("Failed to create tokio runtime: {}", e))
        })?;

        rt.block_on(async {
            // Track start time for duration-limited runs
            let start_time = Instant::now();

            // Set up signal handling
            let running = self.running.clone();
            if let Err(e) = ctrlc::set_handler(move || {
                eprintln!(
                    "{}",
                    "\nCtrl+C received! Shutting down HORUS scheduler...".red()
                );
                if let Ok(mut r) = running.lock() {
                    *r = false;
                }
                std::thread::spawn(|| {
                    std::thread::sleep(std::time::Duration::from_secs(2));
                    eprintln!("{}", "Force terminating - cleaning up session...".red());
                    // Clean up session before forced exit to prevent stale files
                    Self::cleanup_session();
                    std::process::exit(0);
                });
            }) {
                eprintln!("Warning: Failed to set signal handler: {}", e);
            }

            // Set up SIGTERM handler for graceful termination (e.g., from `kill` or `timeout`)
            #[cfg(unix)]
            unsafe {
                libc::signal(libc::SIGTERM, sigterm_handler as libc::sighandler_t);
            }

            // Initialize nodes
            for registered in self.nodes.iter_mut() {
                let node_name = registered.node.name();
                let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

                if should_run && !registered.initialized {
                    if let Some(ref mut ctx) = registered.context {
                        match registered.node.init(ctx) {
                            Ok(()) => {
                                registered.initialized = true;
                                println!("Initialized node '{}'", node_name);
                            }
                            Err(e) => {
                                println!("Failed to initialize node '{}': {}", node_name, e);
                                ctx.transition_to_error(format!("Initialization failed: {}", e));
                            }
                        }
                    }
                }
            }

            // Suppress logging during learning phase for accurate profiling
            // (I/O from logging would skew execution time measurements)
            if !self.learning_complete {
                for registered in self.nodes.iter_mut() {
                    if let Some(ref mut ctx) = registered.context {
                        ctx.set_logging_enabled(false);
                    }
                }
            }

            // Create heartbeat directory
            Self::setup_heartbeat_directory();

            // Write initial registry
            self.update_registry();

            // Build dependency graph from node pub/sub relationships
            self.build_dependency_graph();

            // Main tick loop
            while self.is_running() {
                // Check if duration limit has been reached
                if let Some(max_duration) = duration {
                    if start_time.elapsed() >= max_duration {
                        println!("Scheduler reached time limit of {:?}", max_duration);
                        break;
                    }
                }

                // Check if SIGTERM was received (e.g., from `kill` or `timeout`)
                if SIGTERM_RECEIVED.load(Ordering::SeqCst) {
                    eprintln!(
                        "{}",
                        "\nSIGTERM received! Shutting down HORUS scheduler...".red()
                    );
                    break;
                }

                let now = Instant::now();
                self.last_instant = now;

                // Check if learning phase is complete
                if !self.learning_complete && self.profiler.is_learning_complete() {
                    println!("\n{}", "=== Learning Phase Complete ===".green());

                    // Print profiling statistics
                    self.profiler.print_stats();

                    // Generate tier classification
                    self.classifier = Some(TierClassifier::from_profiler(&self.profiler));

                    // Print classification results
                    if let Some(ref classifier) = self.classifier {
                        classifier.print_classification();

                        let tier_stats = classifier.tier_stats();
                        println!("\nTier Distribution:");
                        println!(
                            "  Ultra-fast nodes: {:.1}%",
                            tier_stats.ultra_fast_percent()
                        );
                        println!(
                            "  Parallel-capable: {:.1}%",
                            tier_stats.parallel_capable_percent()
                        );
                    }

                    // Print node latency percentiles
                    let summary = self.profiler.summary();
                    println!("\nProfiler Summary:");
                    println!("  Total nodes: {}", summary.total_nodes);
                    println!(
                        "  Learning progress: {:.1}%",
                        self.profiler.learning_progress() * 100.0
                    );

                    // Print IO-heavy and CPU-bound nodes
                    let io_nodes = self.profiler.get_io_heavy_nodes();
                    if !io_nodes.is_empty() {
                        println!("  IO-heavy nodes: {:?}", io_nodes);
                    }

                    let cpu_nodes = self.profiler.get_cpu_bound_nodes();
                    if !cpu_nodes.is_empty() {
                        println!("  CPU-bound nodes: {:?}", cpu_nodes);
                    }

                    // Setup JIT compiler for ultra-fast nodes
                    self.setup_jit_compiler();

                    // Restore logging for all nodes BEFORE moving to async executor
                    // This ensures nodes moved to async tier have correct logging state
                    // Tick counts will now start at 0 since logging was disabled during learning
                    for registered in self.nodes.iter_mut() {
                        if let Some(ref mut ctx) = registered.context {
                            ctx.set_logging_enabled(registered.logging_enabled);
                        }
                    }

                    // Initialize async I/O executor and move I/O-heavy nodes
                    // (after logging is restored so moved nodes retain their logging settings)
                    self.setup_async_executor().await;

                    // Setup background executor for low-priority nodes
                    self.setup_background_executor();

                    self.learning_complete = true;
                    println!("{}", "=== Optimization Complete ===\n".green());
                }

                // Execute nodes based on learning phase
                if self.learning_complete {
                    // Optimized execution with parallel groups
                    self.execute_optimized(node_filter).await;
                } else {
                    // Learning mode: sequential execution with profiling
                    self.execute_learning_mode(node_filter).await;
                    self.profiler.tick();
                }

                // Check watchdogs and handle emergency stop for RT systems
                if let Some(ref monitor) = self.safety_monitor {
                    // Check all watchdogs
                    let expired_watchdogs = monitor.check_watchdogs();
                    if !expired_watchdogs.is_empty() {
                        eprintln!(" Watchdog expired for nodes: {:?}", expired_watchdogs);
                    }

                    // Check if emergency stop was triggered
                    if monitor.is_emergency_stop() {
                        eprintln!(" Emergency stop activated - shutting down scheduler");
                        // Record to blackbox
                        if let Some(ref mut bb) = self.blackbox {
                            bb.record(super::blackbox::BlackBoxEvent::EmergencyStop {
                                reason: "Safety monitor triggered emergency stop".to_string(),
                            });
                        }
                        break;
                    }
                }

                // Periodic registry snapshot (every 5 seconds)
                if self.last_snapshot.elapsed() >= Duration::from_secs(5) {
                    self.snapshot_state_to_registry();
                    self.last_snapshot = Instant::now();

                    // Log circuit breaker status for nodes with failures
                    let mut has_breaker_issues = false;
                    for registered in &self.nodes {
                        let stats = registered.circuit_breaker.stats();
                        if stats.failure_count > 0
                            || matches!(stats.state, super::fault_tolerance::CircuitState::Open)
                        {
                            if !has_breaker_issues {
                                println!("\n{}", "Circuit Breaker Status:".yellow());
                                has_breaker_issues = true;
                            }
                            println!(
                                "  {} - State: {:?}, Failures: {}, Successes: {}",
                                registered.node.name(),
                                stats.state,
                                stats.failure_count,
                                stats.success_count
                            );
                        }
                    }

                    // Print dependency graph statistics if available
                    if let Some(ref graph) = self.dependency_graph {
                        if graph.has_cycles() {
                            eprintln!("{}", "WARNING: Dependency graph contains cycles!".red());
                        }
                        let graph_stats = graph.stats();
                        // Print graph stats occasionally
                        use std::sync::atomic::{AtomicU64, Ordering};
                        static GRAPH_LOG_COUNTER: AtomicU64 = AtomicU64::new(0);
                        let count = GRAPH_LOG_COUNTER.fetch_add(1, Ordering::Relaxed) + 1;
                        if count % 100 == 0 {
                            println!(
                                "Dependency Graph - Nodes: {}, Edges: {}, Levels: {}",
                                graph_stats.total_nodes,
                                graph_stats.total_edges,
                                graph_stats.num_levels
                            );
                        }
                    }
                }

                // === Runtime feature integrations ===

                // Black box tick increment
                if let Some(ref mut bb) = self.blackbox {
                    bb.tick();
                }

                // Telemetry export (if interval elapsed)
                if let Some(ref mut tm) = self.telemetry {
                    if tm.should_export() {
                        // Record scheduler metrics - use profiler's learning_ticks as tick count
                        let total_ticks = self
                            .profiler
                            .node_stats
                            .values()
                            .map(|s| s.count)
                            .max()
                            .unwrap_or(0) as u64;
                        tm.counter("scheduler_ticks", total_ticks);
                        tm.gauge("scheduler_uptime_secs", start_time.elapsed().as_secs_f64());
                        tm.gauge("nodes_active", self.nodes.len() as f64);

                        // Record node stats from profiler
                        for registered in &self.nodes {
                            let node_name = registered.node.name();
                            if let Some(stats) = self.profiler.get_stats(node_name) {
                                let mut labels = std::collections::HashMap::new();
                                labels.insert("node".to_string(), node_name.to_string());
                                tm.gauge_with_labels(
                                    "node_avg_duration_us",
                                    stats.avg_us,
                                    labels.clone(),
                                );
                                tm.counter_with_labels(
                                    "node_tick_count",
                                    stats.count as u64,
                                    labels,
                                );
                            }
                        }

                        let _ = tm.export();
                    }
                }

                // Checkpoint creation (if interval elapsed)
                if let Some(ref mut cm) = self.checkpoint_manager {
                    if cm.should_checkpoint() {
                        // Get tick count estimate
                        let total_ticks = self
                            .profiler
                            .node_stats
                            .values()
                            .map(|s| s.count)
                            .max()
                            .unwrap_or(0) as u64;

                        // Create checkpoint metadata
                        let metadata = super::checkpoint::CheckpointMetadata {
                            scheduler_name: self.scheduler_name.clone(),
                            total_ticks,
                            learning_complete: self.learning_complete,
                            node_count: self.nodes.len(),
                            uptime_secs: start_time.elapsed().as_secs_f64(),
                        };

                        // Create checkpoint
                        if let Some(mut checkpoint) = cm.create_checkpoint(metadata) {
                            // Add node states
                            for registered in &self.nodes {
                                let node_name = registered.node.name();
                                let (tick_count, last_tick_us, error_count) = self
                                    .profiler
                                    .get_stats(node_name)
                                    .map(|s| {
                                        (s.count as u64, s.avg_us as u64, s.failure_count as u64)
                                    })
                                    .unwrap_or((0, 0, 0));

                                let node_checkpoint = super::checkpoint::NodeCheckpoint {
                                    name: node_name.to_string(),
                                    tick_count,
                                    last_tick_us,
                                    error_count,
                                    custom_state: None,
                                };
                                checkpoint
                                    .node_states
                                    .insert(node_name.to_string(), node_checkpoint);
                            }

                            if let Err(e) = cm.save_checkpoint(&checkpoint) {
                                eprintln!("[CHECKPOINT] Failed to save: {}", e);
                            }
                        }
                    }
                }

                // Use pre-computed tick period (from config or default ~60Hz)
                tokio::time::sleep(self.tick_period).await;
            }

            // Shutdown async I/O nodes first
            if let Some(ref mut executor) = self.async_io_executor {
                executor.shutdown_all().await;
            }

            // Shutdown nodes
            for registered in self.nodes.iter_mut() {
                let node_name = registered.node.name();
                let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

                if should_run && registered.initialized {
                    if let Some(ref mut ctx) = registered.context {
                        // Write final "Stopped" heartbeat before shutdown - node self-reports
                        ctx.record_shutdown();

                        match registered.node.shutdown(ctx) {
                            Ok(()) => println!("Shutdown node '{}' successfully", node_name),
                            Err(e) => println!("Error shutting down node '{}': {}", node_name, e),
                        }
                    }
                }
            }

            // === Shutdown runtime features ===

            // Shutdown background executor
            if let Some(ref mut executor) = self.background_executor {
                executor.shutdown();
                println!("Background executor shutdown complete");
            }

            // Get total tick count from profiler stats
            let total_ticks = self
                .profiler
                .node_stats
                .values()
                .map(|s| s.count)
                .max()
                .unwrap_or(0) as u64;

            // Record scheduler stop to blackbox and save
            if let Some(ref mut bb) = self.blackbox {
                bb.record(super::blackbox::BlackBoxEvent::SchedulerStop {
                    reason: "Normal shutdown".to_string(),
                    total_ticks,
                });
                if let Err(e) = bb.save() {
                    eprintln!("[BLACKBOX] Failed to save: {}", e);
                }
            }

            // Final telemetry export
            if let Some(ref mut tm) = self.telemetry {
                tm.counter("scheduler_ticks", total_ticks);
                tm.gauge("scheduler_shutdown", 1.0);
                let _ = tm.export();
            }

            // Clean up registry file and session (keep heartbeats for dashboards)
            self.cleanup_registry();
            // Note: Don't cleanup_heartbeats() - let dashboards see final state
            Self::cleanup_session();

            println!("Scheduler shutdown complete");
        });

        Ok(())
    }

    /// Get information about all registered nodes
    pub fn get_node_list(&self) -> Vec<String> {
        self.nodes
            .iter()
            .map(|registered| registered.node.name().to_string())
            .collect()
    }

    /// Get detailed information about a specific node
    pub fn get_node_info(&self, name: &str) -> Option<HashMap<String, String>> {
        for registered in &self.nodes {
            if registered.node.name() == name {
                let mut info = HashMap::new();
                info.insert("name".to_string(), registered.node.name().to_string());
                info.insert("priority".to_string(), registered.priority.to_string());
                info.insert(
                    "logging_enabled".to_string(),
                    registered.logging_enabled.to_string(),
                );
                return Some(info);
            }
        }
        None
    }
    /// Enable/disable logging for a specific node (chainable)
    ///
    /// # Returns
    /// Returns `&mut Self` for method chaining. Logs warning if node not found.
    ///
    /// # Example
    /// ```ignore
    /// scheduler
    ///     .set_node_logging("sensor", false)
    ///     .set_node_logging("controller", true)
    ///     .set_node_rate("motor", 1000.0);
    /// ```
    pub fn set_node_logging(&mut self, name: &str, enabled: bool) -> &mut Self {
        let mut found = false;
        for registered in &mut self.nodes {
            if registered.node.name() == name {
                registered.logging_enabled = enabled;
                println!("Set logging for node '{}' to: {}", name, enabled);
                found = true;
                break;
            }
        }
        if !found {
            eprintln!(
                "Warning: Node '{}' not found for logging configuration",
                name
            );
        }
        self
    }
    /// Get monitoring summary by creating temporary contexts for each node
    pub fn get_monitoring_summary(&self) -> Vec<(String, u32)> {
        self.nodes
            .iter()
            .map(|registered| (registered.node.name().to_string(), registered.priority))
            .collect()
    }

    /// Write metadata to registry file for monitor to read
    fn update_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();

            // Collect pub/sub info from each node
            let nodes_json: Vec<String> = self.nodes.iter().map(|registered| {
                let name = registered.node.name();
                let priority = registered.priority;

                // Get pub/sub from Node trait (macro-declared)
                let mut publishers = registered.node.get_publishers();
                let mut subscribers = registered.node.get_subscribers();

                // Merge runtime-discovered pub/sub from context (if available)
                if let Some(ref ctx) = registered.context {
                    for runtime_pub in ctx.get_registered_publishers() {
                        if !publishers.iter().any(|p| p.topic_name == runtime_pub.topic_name) {
                            publishers.push(runtime_pub);
                        }
                    }
                    for runtime_sub in ctx.get_registered_subscribers() {
                        if !subscribers.iter().any(|s| s.topic_name == runtime_sub.topic_name) {
                            subscribers.push(runtime_sub);
                        }
                    }
                }

                // Format publishers
                let pubs_json = publishers.iter()
                    .map(|p| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        p.topic_name.replace("\"", "\\\""),
                        p.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                // Format subscribers
                let subs_json = subscribers.iter()
                    .map(|s| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        s.topic_name.replace("\"", "\\\""),
                        s.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                format!(
                    "    {{\"name\": \"{}\", \"priority\": {}, \"publishers\": [{}], \"subscribers\": [{}]}}",
                    name, priority, pubs_json, subs_json
                )
            }).collect();

            let registry_data = format!(
                "{{\n  \"pid\": {},\n  \"scheduler_name\": \"{}\",\n  \"working_dir\": \"{}\",\n  \"nodes\": [\n{}\n  ]\n}}",
                pid,
                self.scheduler_name,
                self.working_dir.to_string_lossy(),
                nodes_json.join(",\n")
            );

            let _ = fs::write(&registry_path, registry_data);
        }
    }

    /// Remove registry file when scheduler stops
    fn cleanup_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let _ = fs::remove_file(registry_path);
        }
    }

    /// Get path to registry file
    fn get_registry_path() -> Result<PathBuf, std::io::Error> {
        let mut path = dirs::home_dir().unwrap_or_else(|| PathBuf::from("/tmp"));
        path.push(".horus_registry.json");
        Ok(path)
    }

    /// Create heartbeat directory
    fn setup_heartbeat_directory() {
        // Heartbeats are intentionally global (not session-isolated) so dashboard can monitor all nodes
        let dir = shm_heartbeats_dir();
        let _ = fs::create_dir_all(&dir);
    }

    /// Clean up heartbeat directory (useful for testing/cleanup)
    ///
    /// Note: Not called automatically during shutdown to allow dashboards
    /// to see final node states. Call explicitly if cleanup is needed.
    pub fn cleanup_heartbeats() {
        let dir = shm_heartbeats_dir();
        if dir.exists() {
            let _ = fs::remove_dir_all(&dir);
        }
    }

    /// Clean up session directory (no-op with flat namespace)
    ///
    /// With the simplified flat namespace model, topics are shared globally
    /// and should be cleaned up manually via `horus clean` command.
    fn cleanup_session() {
        // No-op: flat namespace means no session-specific cleanup needed
        // Use `horus clean --shm` to remove all shared memory
    }

    /// Snapshot node state to registry (for crash forensics and persistence)
    /// Called every 5 seconds to avoid I/O overhead
    fn snapshot_state_to_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();
            let timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs();

            // Collect node info including state and health
            let nodes_json: Vec<String> = self.nodes.iter().map(|registered| {
                let name = registered.node.name();
                let priority = registered.priority;

                // Get pub/sub from Node trait (macro-declared)
                let mut publishers = registered.node.get_publishers();
                let mut subscribers = registered.node.get_subscribers();

                // Get state, health, and runtime-discovered pub/sub from context
                let (state_str, health_str, error_count, tick_count) = if let Some(ref ctx) = registered.context {
                    let heartbeat = NodeHeartbeat::from_metrics(
                        ctx.state().clone(),
                        ctx.metrics()
                    );

                    // Merge runtime-discovered pub/sub (from Hub::send/recv with ctx)
                    // These are discovered at runtime when ctx is provided
                    let runtime_pubs = ctx.get_registered_publishers();
                    let runtime_subs = ctx.get_registered_subscribers();
                    for runtime_pub in runtime_pubs {
                        if !publishers.iter().any(|p| p.topic_name == runtime_pub.topic_name) {
                            publishers.push(runtime_pub);
                        }
                    }
                    for runtime_sub in runtime_subs {
                        if !subscribers.iter().any(|s| s.topic_name == runtime_sub.topic_name) {
                            subscribers.push(runtime_sub);
                        }
                    }

                    (
                        ctx.state().to_string(),
                        heartbeat.health.as_str().to_string(),
                        ctx.metrics().errors_count,
                        ctx.metrics().total_ticks,
                    )
                } else {
                    ("Unknown".to_string(), "Unknown".to_string(), 0, 0)
                };

                // Format publishers
                let pubs_json = publishers.iter()
                    .map(|p| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        p.topic_name.replace("\"", "\\\""),
                        p.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                // Format subscribers
                let subs_json = subscribers.iter()
                    .map(|s| format!("{{\"topic\": \"{}\", \"type\": \"{}\"}}",
                        s.topic_name.replace("\"", "\\\""),
                        s.type_name.replace("\"", "\\\"")))
                    .collect::<Vec<_>>()
                    .join(", ");

                format!(
                    "    {{\"name\": \"{}\", \"priority\": {}, \"state\": \"{}\", \"health\": \"{}\", \"error_count\": {}, \"tick_count\": {}, \"publishers\": [{}], \"subscribers\": [{}]}}",
                    name, priority, state_str, health_str, error_count, tick_count, pubs_json, subs_json
                )
            }).collect();

            let registry_data = format!(
                "{{\n  \"pid\": {},\n  \"scheduler_name\": \"{}\",\n  \"working_dir\": \"{}\",\n  \"last_snapshot\": {},\n  \"nodes\": [\n{}\n  ]\n}}",
                pid,
                self.scheduler_name,
                self.working_dir.to_string_lossy(),
                timestamp,
                nodes_json.join(",\n")
            );

            // Atomic write: write to temp file, then rename
            if let Some(parent) = registry_path.parent() {
                let temp_path = parent.join(format!(".horus_registry.json.tmp.{}", pid));

                // Write to temp file
                if fs::write(&temp_path, &registry_data).is_ok() {
                    // Atomically rename to final path
                    let _ = fs::rename(&temp_path, &registry_path);
                }
            }
        }
    }

    /// Build dependency graph from node pub/sub relationships
    fn build_dependency_graph(&mut self) {
        let node_data: Vec<(&str, Vec<String>, Vec<String>)> = self
            .nodes
            .iter()
            .map(|r| {
                let name = r.node.name();
                let pubs = r
                    .node
                    .get_publishers()
                    .iter()
                    .map(|p| p.topic_name.clone())
                    .collect();
                let subs = r
                    .node
                    .get_subscribers()
                    .iter()
                    .map(|s| s.topic_name.clone())
                    .collect();
                (name, pubs, subs)
            })
            .collect();

        if !node_data.is_empty() {
            let graph = DependencyGraph::from_nodes(&node_data);
            self.dependency_graph = Some(graph);
        }
    }

    /// Execute nodes in learning mode (sequential with profiling)
    async fn execute_learning_mode(&mut self, node_filter: Option<&[&str]>) {
        // Sort by priority
        self.nodes.sort_by_key(|r| r.priority);

        // We need to process nodes one at a time to avoid borrow checker issues
        let num_nodes = self.nodes.len();
        for i in 0..num_nodes {
            let (should_run, node_name, should_tick) = {
                let registered = &self.nodes[i];
                let node_name = registered.node.name();
                let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

                // Check rate limiting
                let should_tick = if let Some(rate_hz) = registered.rate_hz {
                    let current_time = Instant::now();
                    if let Some(last_tick) = registered.last_tick {
                        let elapsed_secs = (current_time - last_tick).as_secs_f64();
                        let period_secs = 1.0 / rate_hz;
                        elapsed_secs >= period_secs
                    } else {
                        true
                    }
                } else {
                    true
                };

                (should_run, node_name, should_tick)
            };

            if !should_tick {
                continue;
            }

            // Check circuit breaker
            if !self.nodes[i].circuit_breaker.should_allow() {
                // Circuit is open, skip this node
                continue;
            }

            // Update last tick time if rate limited
            if self.nodes[i].rate_hz.is_some() {
                self.nodes[i].last_tick = Some(Instant::now());
            }

            if should_run && self.nodes[i].initialized {
                // Feed watchdog for RT nodes
                if self.nodes[i].is_rt_node {
                    if let Some(ref monitor) = self.safety_monitor {
                        monitor.feed_watchdog(node_name);
                    }
                }

                let tick_start = Instant::now();
                let tick_result = {
                    let registered = &mut self.nodes[i];
                    if let Some(ref mut context) = registered.context {
                        context.start_tick();

                        // Execute node tick with panic handling
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.tick(Some(context));
                        }))
                    } else {
                        continue;
                    }
                };

                let tick_duration = tick_start.elapsed();

                // Check if node execution failed
                if tick_result.is_err() {
                    // Record failure for Isolated tier classification
                    self.profiler.record_node_failure(node_name);
                    eprintln!("Node '{}' panicked during execution", node_name);
                }

                // Record profiling data
                self.profiler.record(node_name, tick_duration);

                // Check WCET budget for RT nodes
                if self.nodes[i].is_rt_node && self.nodes[i].wcet_budget.is_some() {
                    if let Some(ref monitor) = self.safety_monitor {
                        if let Err(violation) = monitor.check_wcet(node_name, tick_duration) {
                            eprintln!(
                                " WCET violation in {}: {:?} > {:?}",
                                violation.node_name, violation.actual, violation.budget
                            );
                        }
                    }
                }

                // Check deadline for RT nodes
                if self.nodes[i].is_rt_node {
                    if let Some(deadline) = self.nodes[i].deadline {
                        let elapsed = tick_start.elapsed();
                        if elapsed > deadline {
                            if let Some(ref monitor) = self.safety_monitor {
                                monitor.record_deadline_miss(node_name);
                                eprintln!(
                                    " Deadline miss in {}: {:?} > {:?}",
                                    node_name, elapsed, deadline
                                );
                            }
                        }
                    }
                }

                // Handle tick result
                match tick_result {
                    Ok(_) => {
                        // Record success with circuit breaker
                        self.nodes[i].circuit_breaker.record_success();

                        if let Some(ref mut context) = self.nodes[i].context {
                            context.record_tick(); // Node writes its own heartbeat
                        }
                    }
                    Err(panic_err) => {
                        // Record failure with circuit breaker
                        self.nodes[i].circuit_breaker.record_failure();
                        let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                            format!("Node panicked: {}", s)
                        } else if let Some(s) = panic_err.downcast_ref::<String>() {
                            format!("Node panicked: {}", s)
                        } else {
                            "Node panicked with unknown error".to_string()
                        };

                        let registered = &mut self.nodes[i];
                        if let Some(ref mut context) = registered.context {
                            context.record_tick_failure(error_msg.clone()); // Node writes its own heartbeat
                            eprintln!(" {} failed: {}", node_name, error_msg);

                            registered.node.on_error(&error_msg, context);

                            if context.config().restart_on_failure {
                                match context.restart() {
                                    Ok(_) => {
                                        println!(
                                            " Node '{}' restarted successfully (attempt {}/{})",
                                            node_name,
                                            context.metrics().errors_count,
                                            context.config().max_restart_attempts
                                        );
                                        registered.initialized = true;
                                    }
                                    Err(e) => {
                                        eprintln!(
                                            "Node '{}' exceeded max restart attempts: {}",
                                            node_name, e
                                        );
                                        context.transition_to_crashed(format!(
                                            "Max restarts exceeded: {}",
                                            e
                                        ));
                                        registered.initialized = false;
                                    }
                                }
                            } else {
                                context.transition_to_error(error_msg);
                            }
                        }
                    }
                }
            }
        }
    }

    /// Execute nodes in optimized mode (parallel execution based on dependency graph)
    async fn execute_optimized(&mut self, node_filter: Option<&[&str]>) {
        // If no dependency graph available, fall back to sequential
        if self.dependency_graph.is_none() {
            self.execute_learning_mode(node_filter).await;
            return;
        }

        // Trigger async I/O nodes
        if let Some(ref executor) = self.async_io_executor {
            executor.tick_all().await;
        }

        // Trigger background nodes (low-priority, non-blocking)
        if let Some(ref executor) = self.background_executor {
            executor.tick_all();
        }

        // Execute nodes level by level (nodes in same level can run in parallel)
        let levels = self
            .dependency_graph
            .as_ref()
            .expect("Dependency graph should exist - checked above")
            .levels
            .clone();

        for level in &levels {
            // Find indices of nodes in this level that should run
            let mut level_indices = Vec::new();

            for node_name in level {
                for (idx, registered) in self.nodes.iter().enumerate() {
                    if registered.node.name() == node_name {
                        let should_run =
                            node_filter.is_none_or(|filter| filter.contains(&node_name.as_str()));

                        // Check rate limiting
                        let should_tick = if let Some(rate_hz) = registered.rate_hz {
                            let current_time = Instant::now();
                            if let Some(last_tick) = registered.last_tick {
                                let elapsed_secs = (current_time - last_tick).as_secs_f64();
                                let period_secs = 1.0 / rate_hz;
                                elapsed_secs >= period_secs
                            } else {
                                true
                            }
                        } else {
                            true
                        };

                        if should_run && registered.initialized && should_tick {
                            level_indices.push(idx);
                        }
                        break;
                    }
                }
            }

            // Execute nodes in this level
            // NOTE: True parallel execution requires refactoring to allow concurrent
            // mutable access to different Vec elements. Options:
            // 1. Use UnsafeCell/RwLock per node (adds overhead)
            // 2. Restructure nodes into separate Vecs (breaks encapsulation)
            // 3. Use async/await with message passing (architectural change)
            //
            // For now, execute level-by-level sequentially. Since levels are already
            // topologically sorted, this ensures correctness. Parallelism benefit
            // would only apply within levels with multiple independent nodes.
            //
            // Performance: Still better than original sequential-by-priority because:
            // - Respects true dependencies (not just priority)
            // - Enables future parallelization without API changes
            // - Critical path optimization from dependency analysis
            for idx in level_indices {
                self.execute_single_node(idx);
            }
        }

        // Process any async I/O results
        self.process_async_results().await;

        // Process any background executor results (non-blocking)
        self.process_background_results();
    }

    /// Execute a single node by index with RT support
    fn execute_single_node(&mut self, idx: usize) {
        // Check circuit breaker first
        if !self.nodes[idx].circuit_breaker.should_allow() {
            // Circuit is open, skip this node
            return;
        }

        // Update rate limit timestamp
        if self.nodes[idx].rate_hz.is_some() {
            self.nodes[idx].last_tick = Some(Instant::now());
        }

        let node_name = self.nodes[idx].node.name();
        let is_rt_node = self.nodes[idx].is_rt_node;
        let wcet_budget = self.nodes[idx].wcet_budget;
        let deadline = self.nodes[idx].deadline;

        // Feed watchdog for RT nodes
        if is_rt_node {
            if let Some(ref monitor) = self.safety_monitor {
                monitor.feed_watchdog(node_name);
            }
        }

        let tick_start = Instant::now();

        // Check if this node should use JIT execution path
        let use_jit_path = self.nodes[idx].is_jit_compiled && self.nodes[idx].jit_stats.is_some();

        let (tick_result, jit_executed) = if use_jit_path {
            // JIT EXECUTION PATH: Use compiled native code for ultra-fast execution
            let registered = &mut self.nodes[idx];
            if let Some(ref mut context) = registered.context {
                context.start_tick();
            }

            // Try to execute via JIT-compiled function
            let jit_result = if let Some(ref mut jit_stats) = registered.jit_stats {
                if !jit_stats.func_ptr.is_null() {
                    // Execute the JIT-compiled function
                    // Use a simple incrementing input for demonstration
                    // In real usage, nodes would provide their own input mechanism
                    let input = jit_stats.exec_count as i64;
                    let _result = jit_stats.execute(input);
                    true
                } else {
                    false
                }
            } else {
                false
            };

            if jit_result {
                // JIT execution succeeded
                (Ok(()), true)
            } else {
                // JIT failed, fall back to regular tick
                let tick_res = if let Some(ref mut context) = registered.context {
                    std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                        registered.node.tick(Some(context));
                    }))
                } else {
                    return;
                };
                (tick_res, false)
            }
        } else {
            // REGULAR EXECUTION PATH: Standard node tick
            let registered = &mut self.nodes[idx];
            if let Some(ref mut context) = registered.context {
                context.start_tick();

                let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                    // Check if node provides a direct JIT compute function
                    if let Some(compute_fn) = registered.node.get_jit_compute() {
                        // Execute the function pointer directly
                        let _result = compute_fn(0);
                    } else {
                        // Regular tick execution
                        registered.node.tick(Some(context));
                    }
                }));
                (result, false)
            } else {
                return;
            }
        };

        let tick_duration = tick_start.elapsed();

        // Check if node execution failed
        if tick_result.is_err() {
            // Record failure for Isolated tier classification
            self.profiler.record_node_failure(node_name);
            eprintln!("Node '{}' panicked during execution", node_name);
        }

        self.profiler.record(node_name, tick_duration);

        // Update JIT compilation statistics if this is a JIT-compiled node
        if self.nodes[idx].is_jit_compiled {
            // Update JIT execution statistics
            if let Some(ref mut jit_stats) = self.nodes[idx].jit_stats {
                // Stats are already updated by execute() call if JIT path was used
                if !jit_executed {
                    // Only update manually if we didn't use JIT path
                    jit_stats.exec_count += 1;
                    jit_stats.total_ns += tick_duration.as_nanos() as u64;
                }

                // Log performance periodically
                if jit_stats.exec_count % 1000 == 0 {
                    let avg_ns = jit_stats.avg_exec_ns();
                    let is_fast = jit_stats.is_fast_enough();
                    println!(
                        "[JIT] Node '{}' - {} executions, avg: {:.0}ns (target: 20-50ns) {} {}",
                        node_name,
                        jit_stats.exec_count,
                        avg_ns,
                        if jit_executed { "[NATIVE]" } else { "[TICK]" },
                        if is_fast { "✓" } else { "SLOW" }
                    );
                }
            }

            // Also update in the global JIT map
            if let Some(compiled) = self.jit_compiled_nodes.get_mut(node_name) {
                compiled.exec_count += 1;
                compiled.total_ns += tick_duration.as_nanos() as u64;
            }
        }

        // Check WCET budget for RT nodes
        if is_rt_node && wcet_budget.is_some() {
            if let Some(ref monitor) = self.safety_monitor {
                if let Err(violation) = monitor.check_wcet(node_name, tick_duration) {
                    eprintln!(
                        " WCET violation in {}: {:?} > {:?}",
                        violation.node_name, violation.actual, violation.budget
                    );
                }
            }
        }

        // Check deadline for RT nodes
        if is_rt_node {
            if let Some(deadline_duration) = deadline {
                let elapsed = tick_start.elapsed();
                if elapsed > deadline_duration {
                    if let Some(ref monitor) = self.safety_monitor {
                        monitor.record_deadline_miss(node_name);
                        eprintln!(
                            " Deadline miss in {}: {:?} > {:?}",
                            node_name, elapsed, deadline_duration
                        );
                    }
                }
            }
        }

        match tick_result {
            Ok(_) => {
                // Record success with circuit breaker
                self.nodes[idx].circuit_breaker.record_success();

                if let Some(ref mut context) = self.nodes[idx].context {
                    context.record_tick(); // Node writes its own heartbeat
                }
            }
            Err(panic_err) => {
                // Record failure with circuit breaker
                self.nodes[idx].circuit_breaker.record_failure();
                let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
                    format!("Node panicked: {}", s)
                } else if let Some(s) = panic_err.downcast_ref::<String>() {
                    format!("Node panicked: {}", s)
                } else {
                    "Node panicked with unknown error".to_string()
                };

                let registered = &mut self.nodes[idx];
                if let Some(ref mut context) = registered.context {
                    context.record_tick_failure(error_msg.clone()); // Node writes its own heartbeat
                    eprintln!(" {} failed: {}", node_name, error_msg);

                    registered.node.on_error(&error_msg, context);

                    if context.config().restart_on_failure {
                        match context.restart() {
                            Ok(_) => {
                                println!(
                                    " Node '{}' restarted successfully (attempt {}/{})",
                                    node_name,
                                    context.metrics().errors_count,
                                    context.config().max_restart_attempts
                                );
                                registered.initialized = true;
                            }
                            Err(e) => {
                                eprintln!(
                                    "Node '{}' exceeded max restart attempts: {}",
                                    node_name, e
                                );
                                context
                                    .transition_to_crashed(format!("Max restarts exceeded: {}", e));
                                registered.initialized = false;
                            }
                        }
                    } else {
                        context.transition_to_error(error_msg);
                    }
                }
            }
        }
    }

    /// Setup JIT compiler for ultra-fast nodes
    fn setup_jit_compiler(&mut self) {
        // Identify ultra-fast nodes from classifier
        if let Some(ref classifier) = self.classifier {
            let mut jit_compiled_count = 0;
            let mut already_compiled_count = 0;

            // Automatically JIT-compile ultra-fast nodes
            for i in 0..self.nodes.len() {
                let node_name = self.nodes[i].node.name();

                if let Some(tier) = classifier.get_tier(node_name) {
                    if tier == ExecutionTier::UltraFast {
                        // Skip if already JIT-compiled at add-time with proper params
                        if self.nodes[i].is_jit_compiled && self.nodes[i].jit_stats.is_some() {
                            already_compiled_count += 1;
                            println!(
                                "[JIT] Node '{}' already compiled at add-time (keeping original)",
                                node_name
                            );
                            continue;
                        }

                        // Try to compile using node's JIT params if available
                        let compiled = if let Some((factor, offset)) =
                            self.nodes[i].node.get_jit_arithmetic_params()
                        {
                            // Use node's actual parameters
                            match super::jit::JITCompiler::new() {
                                Ok(mut compiler) => {
                                    let unique_name = format!("{}_{}_learning", node_name, i);
                                    match compiler.compile_arithmetic_node(
                                        &unique_name,
                                        factor,
                                        offset,
                                    ) {
                                        Ok(func_ptr) => {
                                            println!(
                                                "[JIT] Learning phase compiled '{}' with factor={}, offset={}",
                                                node_name, factor, offset
                                            );
                                            Some(CompiledDataflow {
                                                name: node_name.to_string(),
                                                func_ptr,
                                                exec_count: 0,
                                                total_ns: 0,
                                            })
                                        }
                                        Err(_) => Some(CompiledDataflow::new_stats_only(node_name)),
                                    }
                                }
                                Err(_) => Some(CompiledDataflow::new_stats_only(node_name)),
                            }
                        } else {
                            // No JIT params - use generic function for stats tracking
                            Some(CompiledDataflow::new_stats_only(node_name))
                        };

                        if let Some(compiled) = compiled {
                            // Mark this node as JIT-compiled
                            self.nodes[i].is_jit_compiled = true;

                            // Store the compiled dataflow for tracking
                            self.jit_compiled_nodes.insert(
                                node_name.to_string(),
                                CompiledDataflow {
                                    name: compiled.name.clone(),
                                    func_ptr: compiled.func_ptr,
                                    exec_count: 0,
                                    total_ns: 0,
                                },
                            );
                            self.nodes[i].jit_stats = Some(compiled);

                            jit_compiled_count += 1;
                            println!(
                                "[JIT] Auto-compiled node '{}' for ultra-fast execution (target: 20-50ns)",
                                node_name
                            );
                        }
                    }
                }
            }

            if jit_compiled_count > 0 || already_compiled_count > 0 {
                println!(
                    "[JIT] {} nodes auto-compiled, {} already compiled at add-time",
                    jit_compiled_count, already_compiled_count
                );
            }
        }
    }

    /// Setup async executor and move I/O-heavy nodes to it
    async fn setup_async_executor(&mut self) {
        // Create async I/O executor
        let mut async_executor = match AsyncIOExecutor::new() {
            Ok(exec) => exec,
            Err(_) => return, // Continue without async tier if creation fails
        };

        // Create channel for async results
        let (tx, rx) = mpsc::unbounded_channel();
        self.async_result_tx = Some(tx.clone());
        self.async_result_rx = Some(rx);

        // Identify I/O-heavy nodes from classifier
        if let Some(ref classifier) = self.classifier {
            let mut nodes_to_move = Vec::new();

            // Find indices of I/O-heavy nodes
            for (idx, registered) in self.nodes.iter().enumerate() {
                let node_name = registered.node.name();

                // Check if this node is classified as AsyncIO tier
                if let Some(tier) = classifier.get_tier(node_name) {
                    if tier == ExecutionTier::AsyncIO {
                        nodes_to_move.push(idx);
                    }
                }
            }

            // Move nodes to async executor (in reverse order to maintain indices)
            for idx in nodes_to_move.into_iter().rev() {
                // Remove from main scheduler
                let registered = self.nodes.swap_remove(idx);
                let node_name = registered.node.name().to_string();

                // Spawn in async executor
                if let Err(e) =
                    async_executor.spawn_node(registered.node, registered.context, tx.clone())
                {
                    eprintln!("Failed to move {} to async tier: {}", node_name, e);
                    // Note: Can't put it back since we've moved ownership
                    // This is acceptable as the node would be dropped anyway
                }
            }
        }

        self.async_io_executor = Some(async_executor);
    }

    /// Process async I/O results
    async fn process_async_results(&mut self) {
        if let Some(ref mut rx) = self.async_result_rx {
            // Process all available results without blocking
            while let Ok(result) = rx.try_recv() {
                if !result.success {
                    if let Some(ref error) = result.error {
                        eprintln!("Async node {} failed: {}", result.node_name, error);
                    }
                }
            }
        }
    }

    /// Setup background executor and move background-tier nodes to it
    fn setup_background_executor(&mut self) {
        // Create background executor
        let mut bg_executor = match BackgroundExecutor::new() {
            Ok(exec) => exec,
            Err(e) => {
                eprintln!("[Background] Failed to create executor: {}", e);
                return;
            }
        };

        // Identify background nodes from classifier
        if let Some(ref classifier) = self.classifier {
            let mut nodes_to_move = Vec::new();

            // Find indices of background nodes
            for (idx, registered) in self.nodes.iter().enumerate() {
                let node_name = registered.node.name();

                // Check if this node is classified as Background tier
                if let Some(tier) = classifier.get_tier(node_name) {
                    if tier == ExecutionTier::Background {
                        nodes_to_move.push(idx);
                    }
                }
            }

            // Move nodes to background executor (in reverse order to maintain indices)
            for idx in nodes_to_move.into_iter().rev() {
                // Remove from main scheduler
                let registered = self.nodes.swap_remove(idx);
                let node_name = registered.node.name().to_string();

                // Spawn in background executor
                if let Err(e) = bg_executor.spawn_node(registered.node, registered.context) {
                    eprintln!("Failed to move {} to background tier: {}", node_name, e);
                }
            }

            if bg_executor.node_count() > 0 {
                println!(
                    "[Background] Moved {} nodes to low-priority thread",
                    bg_executor.node_count()
                );
            }
        }

        self.background_executor = Some(bg_executor);
    }

    /// Process background executor results (non-blocking)
    fn process_background_results(&mut self) {
        if let Some(ref executor) = self.background_executor {
            for result in executor.poll_results() {
                if !result.success {
                    if let Some(ref error) = result.error {
                        eprintln!("[Background] Node {} failed: {}", result.node_name, error);
                    }
                }
            }
        }
    }

    /// Configure the scheduler for specific robot types (runtime configuration)
    ///
    /// **Note**: For builder pattern during construction, use `with_config()` instead.
    /// This method is for runtime reconfiguration of an existing scheduler.
    ///
    /// # Examples
    ///
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::SchedulerConfig;
    /// // Runtime reconfiguration
    /// let mut scheduler = Scheduler::new();
    /// #[allow(deprecated)]
    /// scheduler.set_config(SchedulerConfig::hard_realtime());
    /// ```
    ///
    /// # Prefer Builder Pattern
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::SchedulerConfig;
    /// // Better: Use with_config() during construction
    /// let scheduler = Scheduler::new()
    ///     .with_config(SchedulerConfig::hard_realtime());
    /// ```
    #[deprecated(
        since = "0.2.0",
        note = "Use with_config() for builder pattern. set_config() is only for runtime reconfiguration."
    )]
    pub fn set_config(&mut self, config: super::config::SchedulerConfig) -> &mut Self {
        use super::config::*;

        // Apply execution mode
        match config.execution {
            ExecutionMode::JITOptimized => {
                // Force JIT compilation for all nodes
                self.profiler.force_ultra_fast_classification = true;
                println!("JIT optimization mode selected");
            }
            ExecutionMode::Parallel => {
                // Enable full parallelization
                self.parallel_executor.set_max_threads(num_cpus::get());
                println!("Parallel execution mode selected");
            }
            ExecutionMode::AsyncIO => {
                // Force async I/O tier for all I/O operations
                self.profiler.force_async_io_classification = true;
                println!("Async I/O mode selected");
            }
            ExecutionMode::Sequential => {
                // Disable all optimizations for deterministic execution
                self.learning_complete = true; // Skip learning phase
                self.classifier = None;
                self.parallel_executor.set_max_threads(1);
                println!("Sequential execution mode selected");
            }
            ExecutionMode::AutoAdaptive => {
                // Default adaptive behavior
                println!("Auto-adaptive mode selected");
            }
        }

        // Apply real-time configuration
        if config.realtime.safety_monitor
            || config.realtime.wcet_enforcement
            || config.realtime.deadline_monitoring
        {
            // Create safety monitor with configured deadline miss limit
            let mut monitor = SafetyMonitor::new(config.realtime.max_deadline_misses);

            // Configure critical nodes and WCET budgets for RT nodes
            for registered in self.nodes.iter() {
                if registered.is_rt_node {
                    let node_name = registered.node.name().to_string();

                    // Add as critical node with watchdog if configured
                    if config.realtime.watchdog_enabled {
                        let watchdog_timeout =
                            Duration::from_millis(config.realtime.watchdog_timeout_ms);
                        monitor.add_critical_node(node_name.clone(), watchdog_timeout);
                    }

                    // Set WCET budget if available
                    if let Some(wcet) = registered.wcet_budget {
                        monitor.set_wcet_budget(node_name, wcet);
                    }
                }
            }

            self.safety_monitor = Some(monitor);
            println!("Safety monitor configured for RT nodes");
        }

        // Apply timing configuration
        if config.timing.per_node_rates {
            // Per-node rate control already supported via set_node_rate()
        }

        // Global rate control
        let _tick_period_ms = (1000.0 / config.timing.global_rate_hz) as u64;
        // This will be used in the run loop (store for later)

        // Apply fault tolerance
        for registered in self.nodes.iter_mut() {
            if config.fault.circuit_breaker_enabled {
                registered.circuit_breaker = CircuitBreaker::new(
                    config.fault.max_failures,
                    config.fault.recovery_threshold,
                    config.fault.circuit_timeout_ms,
                );
            } else {
                // Disable circuit breaker by setting impossibly high threshold
                registered.circuit_breaker = CircuitBreaker::new(u32::MAX, 0, 0);
            }
        }

        // Apply resource configuration
        if let Some(ref cores) = config.resources.cpu_cores {
            // Set CPU affinity
            self.parallel_executor.set_cpu_cores(cores.clone());
            println!("CPU cores configuration: {:?}", cores);
        }

        // Apply monitoring configuration
        if config.monitoring.profiling_enabled {
            self.profiler.enable();
            println!("Profiling enabled");
        } else {
            self.profiler.disable();
            println!("Profiling disabled");
        }

        // Handle robot presets
        match config.preset {
            RobotPreset::SafetyCritical => {
                println!("Configured for safety-critical operation");
            }
            RobotPreset::HardRealTime => {
                println!("Configured for hard real-time operation");
            }
            RobotPreset::HighPerformance => {
                println!("Configured for high-performance operation");
            }
            RobotPreset::Space => {
                println!("Configured for space robotics");
            }
            RobotPreset::Swarm => {
                println!("Configured for swarm robotics");
                // Apply swarm-specific settings
                if let Some(swarm_id) = config.get_custom::<i64>("swarm_id") {
                    self.scheduler_name = format!("Swarm_{}", swarm_id);
                }
            }
            RobotPreset::SoftRobotics => {
                println!("Configured for soft robotics");
            }
            RobotPreset::Custom => {
                println!("Using custom configuration");
            }
            _ => {
                // Standard preset
            }
        }

        // === Apply new runtime features ===

        // 1. Global tick rate enforcement
        self.tick_period =
            std::time::Duration::from_micros((1_000_000.0 / config.timing.global_rate_hz) as u64);

        // 2. Checkpoint system
        if config.fault.checkpoint_interval_ms > 0 {
            let checkpoint_dir = std::path::PathBuf::from("/tmp/horus_checkpoints");
            let cm = super::checkpoint::CheckpointManager::new(
                checkpoint_dir,
                config.fault.checkpoint_interval_ms,
            );
            self.checkpoint_manager = Some(cm);
            println!(
                "[SCHEDULER] Checkpoint system enabled (interval: {}ms)",
                config.fault.checkpoint_interval_ms
            );
        }

        // 3. Black box flight recorder
        if config.monitoring.black_box_enabled && config.monitoring.black_box_size_mb > 0 {
            let mut bb = super::blackbox::BlackBox::new(config.monitoring.black_box_size_mb);
            bb.record(super::blackbox::BlackBoxEvent::SchedulerStart {
                name: self.scheduler_name.clone(),
                node_count: self.nodes.len(),
                config: format!("{:?}", config.preset),
            });
            self.blackbox = Some(bb);
            println!(
                "[SCHEDULER] Black box enabled ({}MB buffer)",
                config.monitoring.black_box_size_mb
            );
        }

        // 4. Telemetry endpoint
        if let Some(ref endpoint_str) = config.monitoring.telemetry_endpoint {
            let endpoint = super::telemetry::TelemetryEndpoint::from_string(endpoint_str);
            let interval_ms = config.monitoring.metrics_interval_ms;
            let mut tm = super::telemetry::TelemetryManager::new(endpoint, interval_ms);
            tm.set_scheduler_name(&self.scheduler_name);
            self.telemetry = Some(tm);
            println!("[SCHEDULER] Telemetry enabled (endpoint: {})", endpoint_str);
        }

        // 5. Redundancy (TMR)
        if config.fault.redundancy_factor > 1 {
            // Default to majority voting strategy
            let strategy = super::redundancy::VotingStrategy::Majority;
            self.redundancy = Some(super::redundancy::RedundancyManager::new(
                config.fault.redundancy_factor as usize,
                strategy,
            ));
            println!(
                "[SCHEDULER] Redundancy enabled (factor: {}, strategy: {:?})",
                config.fault.redundancy_factor, strategy
            );
        }

        // 6. Real-time optimizations (Linux-specific)
        #[cfg(target_os = "linux")]
        {
            // Memory locking
            if config.realtime.memory_locking && super::runtime::lock_all_memory().is_ok() {
                println!("[SCHEDULER] Memory locked (mlockall)");
            }

            // RT scheduling class
            if config.realtime.rt_scheduling_class {
                let priority = 50; // Default RT priority
                if super::runtime::set_realtime_priority(priority).is_ok() {
                    println!(
                        "[SCHEDULER] RT scheduling enabled (SCHED_FIFO, priority {})",
                        priority
                    );
                }
            }

            // CPU core affinity
            if let Some(ref cores) = config.resources.cpu_cores {
                if super::runtime::set_thread_affinity(cores).is_ok() {
                    println!("[SCHEDULER] CPU affinity set to cores {:?}", cores);
                }
            }

            // NUMA awareness
            if config.resources.numa_aware {
                let numa_nodes = super::runtime::get_numa_node_count();
                if numa_nodes > 1 {
                    println!(
                        "[SCHEDULER] NUMA-aware scheduling ({} nodes detected)",
                        numa_nodes
                    );
                }
            }
        }

        // 7. Recording configuration for record/replay system
        if let Some(ref recording_yaml) = config.recording {
            if recording_yaml.enabled {
                // Generate session name if not provided
                let session_name = recording_yaml.session_name.clone().unwrap_or_else(|| {
                    use std::time::{SystemTime, UNIX_EPOCH};
                    let timestamp = SystemTime::now()
                        .duration_since(UNIX_EPOCH)
                        .map(|d| d.as_secs())
                        .unwrap_or(0);
                    format!("session_{}", timestamp)
                });

                // Convert YAML config to internal RecordingConfig
                let mut recording_config = RecordingConfig::new(session_name.clone());
                recording_config.compress = recording_yaml.compress;
                recording_config.interval = recording_yaml.interval as u64;

                if let Some(ref output_dir) = recording_yaml.output_dir {
                    recording_config.base_dir = PathBuf::from(output_dir);
                }

                // Store include/exclude filters in the config
                recording_config.include_nodes = recording_yaml.include_nodes.clone();
                recording_config.exclude_nodes = recording_yaml.exclude_nodes.clone();

                // Enable recording
                self.recording_config = Some(recording_config.clone());

                // Generate unique scheduler ID
                let scheduler_id = format!(
                    "{:x}{:x}",
                    std::time::SystemTime::now()
                        .duration_since(std::time::UNIX_EPOCH)
                        .map(|d| d.as_nanos() as u64)
                        .unwrap_or(0),
                    std::process::id() as u64
                );

                // Create scheduler-level recording
                self.scheduler_recording =
                    Some(SchedulerRecording::new(&scheduler_id, &session_name));

                println!(
                    "[SCHEDULER] Recording enabled (session: {}, compress: {})",
                    session_name, recording_yaml.compress
                );
            }
        }

        // Store config for runtime use
        self.config = Some(config);

        self
    }
}
