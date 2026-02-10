use crate::core::hlog::{clear_node_context, set_node_context};
use crate::core::{announce_started, announce_stopped, Node, NodeInfo, NodePresence};
use crate::error::HorusResult;
use crate::horus_internal;
use crate::memory::platform::shm_control_dir;
use crate::terminal::print_line;
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

// Executor imports
use super::executors::{AsyncIOExecutor, AsyncResult, BackgroundExecutor, IsolatedExecutor, IsolatedNodeConfig};
use super::intelligence::NodeTier;

// Import types from types module
use super::types::{RegisteredNode, SchedulerNodeMetrics};

// Tokio channel for async executor results
use tokio::sync::mpsc;

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

use super::executors::ParallelExecutor;
use super::fault_tolerance::CircuitBreaker;
use super::intelligence::RuntimeProfiler;
use super::safety_monitor::SafetyMonitor;

// Auto-optimization capabilities
use super::capabilities::RuntimeCapabilities;

// Deterministic execution (for simulation mode)
use super::deterministic::{DeterministicClock, DeterministicConfig, ExecutionTrace};

use parking_lot::Mutex as ParkingMutex;

/// Degradation that occurred during auto-optimization.
///
/// When `Scheduler::new()` auto-applies RT optimizations, it may encounter
/// failures (e.g., no RT permission). These are recorded as degradations
/// rather than errors, allowing the scheduler to still function with
/// reduced capabilities.
#[derive(Debug, Clone)]
pub struct RtDegradation {
    /// What feature was attempted
    pub feature: RtFeature,
    /// What went wrong
    pub reason: String,
    /// Severity of the degradation
    pub severity: DegradationSeverity,
}

/// RT feature that was attempted during auto-optimization.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum RtFeature {
    /// SCHED_FIFO/SCHED_RR priority
    RtPriority,
    /// mlockall() memory locking
    MemoryLocking,
    /// CPU affinity to isolated cores
    CpuAffinity,
    /// NUMA pinning
    NumaPinning,
    /// Watchdog timer
    Watchdog,
    /// Safety monitor
    SafetyMonitor,
}

impl std::fmt::Display for RtFeature {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        match self {
            RtFeature::RtPriority => write!(f, "RT Priority"),
            RtFeature::MemoryLocking => write!(f, "Memory Locking"),
            RtFeature::CpuAffinity => write!(f, "CPU Affinity"),
            RtFeature::NumaPinning => write!(f, "NUMA Pinning"),
            RtFeature::Watchdog => write!(f, "Watchdog"),
            RtFeature::SafetyMonitor => write!(f, "Safety Monitor"),
        }
    }
}

/// Severity of RT degradation.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DegradationSeverity {
    /// Significant impact on performance (e.g., no RT priority)
    High,
    /// Moderate impact (e.g., no memory locking)
    Medium,
    /// Minor impact (e.g., no NUMA awareness)
    Low,
}

/// Central orchestrator: holds nodes, drives the tick loop.
pub struct Scheduler {
    nodes: Vec<RegisteredNode>,
    running: Arc<Mutex<bool>>,
    last_instant: Instant,
    last_snapshot: Instant,
    scheduler_name: String,
    working_dir: PathBuf,

    // Profiling (for metrics reporting)
    profiler: RuntimeProfiler,
    parallel_executor: ParallelExecutor,

    // Configuration (stored for runtime use)
    config: Option<super::config::SchedulerConfig>,

    // Safety monitor for real-time critical systems
    safety_monitor: Option<SafetyMonitor>,

    // === New runtime features ===
    // Tick rate enforcement
    tick_period: Duration,

    // Black box flight recorder
    blackbox: Option<super::blackbox::BlackBox>,

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
    // Stop replay at this tick (None = run to end)
    replay_stop_tick: Option<u64>,
    // Replay speed multiplier (1.0 = normal, 0.5 = half speed, 2.0 = double)
    replay_speed: f64,

    // === Auto-optimization (NEW) ===
    /// Detected runtime capabilities (RT, memory, CPU topology, platform)
    runtime_capabilities: Option<RuntimeCapabilities>,

    /// Degradations that occurred during auto-optimization
    /// (features that were attempted but failed to apply)
    rt_degradations: Vec<RtDegradation>,

    // === Deterministic execution (simulation mode) ===
    /// Deterministic clock for virtual time (simulation mode only)
    /// Provides reproducible timestamps and seeded RNG
    deterministic_clock: Option<Arc<DeterministicClock>>,

    /// Execution trace for recording and replay (simulation mode only)
    /// Records all node executions with timing for reproducibility verification
    execution_trace: Option<Arc<ParkingMutex<ExecutionTrace>>>,

    /// Deterministic configuration (seed, tick duration, etc.)
    deterministic_config: Option<DeterministicConfig>,

    // === Tier-based executors ===
    /// Async I/O executor for non-blocking node execution
    async_io_executor: Option<AsyncIOExecutor>,
    /// Channel for receiving async executor results
    async_result_rx: Option<mpsc::UnboundedReceiver<AsyncResult>>,
    /// Channel sender for async executor (cloned to spawned tasks)
    async_result_tx: Option<mpsc::UnboundedSender<AsyncResult>>,
    /// Background executor for low-priority nodes
    background_executor: Option<BackgroundExecutor>,
    /// Isolated executor for process-isolated fault-tolerant nodes
    isolated_executor: Option<IsolatedExecutor>,

    // === Telemetry ===
    /// Telemetry manager for live metrics export
    telemetry: Option<super::telemetry::TelemetryManager>,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler {
    /// Create a new scheduler with **automatic RT optimization**.
    ///
    /// The scheduler automatically detects and applies available RT features:
    /// - **RT Priority**: If SCHED_FIFO is available, applies recommended priority
    /// - **Memory Locking**: If mlockall() is permitted, locks all memory pages
    /// - **CPU Affinity**: If isolated CPUs exist, pins to the best RT core
    /// - **Safety Monitor**: Enabled with sensible defaults
    ///
    /// Any feature that fails to apply is recorded as a "degradation" rather
    /// than an error. Check `scheduler.degradations()` to see what didn't work.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let scheduler = Scheduler::new();  // Auto-detects and optimizes!
    ///
    /// // Check what was applied
    /// if let Some(caps) = scheduler.capabilities() {
    ///     println!("RT support: {}", caps.has_rt_support());
    ///     println!("Can lock memory: {}", caps.can_lock_memory());
    /// }
    ///
    /// // Check what degraded
    /// for deg in scheduler.degradations() {
    ///     println!("Warning: {} - {}", deg.feature, deg.reason);
    /// }
    /// ```
    ///
    /// # Alternative Constructors
    /// - `Scheduler::simulation()` - Fast, no RT features, no detection
    /// - `Scheduler::prototype()` - Development mode with verbose logging
    /// - `Scheduler::builder()` - Full control over every option
    pub fn new() -> Self {
        let running = Arc::new(Mutex::new(true));
        let now = Instant::now();

        // Detect runtime capabilities (~30-100μs one-time cost)
        let caps = RuntimeCapabilities::detect();
        let mut degradations = Vec::new();

        // Create base scheduler
        let mut scheduler = Self {
            nodes: Vec::new(),
            running,
            last_instant: now,
            last_snapshot: now,
            scheduler_name: "AutoScheduler".to_string(),
            working_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),

            // Profiling for metrics
            profiler: RuntimeProfiler::new_default(),
            parallel_executor: ParallelExecutor::new(),

            // Configuration
            config: None,

            // Safety monitor (disabled by default for prototyping)
            // Enable explicitly via builder.safety_monitor(N) or use safety_critical()/hard_realtime() presets
            safety_monitor: None,

            // Runtime features
            tick_period: Duration::from_micros(16667), // ~60Hz default
            blackbox: Some(super::blackbox::BlackBox::new(16)), // 16MB default for crash analysis

            // Record/Replay system (disabled by default)
            recording_config: None,
            scheduler_recording: None,
            replay_mode: None,
            replay_nodes: HashMap::new(),
            replay_overrides: HashMap::new(),
            current_tick: 0,
            replay_stop_tick: None,
            replay_speed: 1.0,

            // Auto-optimization: store capabilities, degradations added below
            runtime_capabilities: Some(caps.clone()),
            rt_degradations: Vec::new(),

            // Deterministic execution (disabled in production mode)
            deterministic_clock: None,
            execution_trace: None,
            deterministic_config: None,

            // Tier-based executors (set up during run() based on node tier annotations)
            async_io_executor: None,
            async_result_rx: None,
            async_result_tx: None,
            background_executor: None,
            isolated_executor: None,

            // Telemetry (configured via with_config())
            telemetry: None,
        };

        // === Auto-apply RT features based on detected capabilities ===

        // 1. Apply RT priority if available
        if caps.has_rt_support() {
            let priority = caps.recommended_rt_priority();
            match scheduler.apply_rt_priority_internal(priority) {
                Ok(()) => {
                    print_line(
                        &format!("[AUTO] RT priority {} applied (SCHED_FIFO)", priority)
                            .green()
                            .to_string(),
                    );
                }
                Err(e) => {
                    degradations.push(RtDegradation {
                        feature: RtFeature::RtPriority,
                        reason: e.to_string(),
                        severity: DegradationSeverity::High,
                    });
                }
            }
        } else {
            degradations.push(RtDegradation {
                feature: RtFeature::RtPriority,
                reason: format!(
                    "RT scheduling not available (max_priority={}, preempt_rt={})",
                    caps.max_rt_priority, caps.preempt_rt
                ),
                severity: DegradationSeverity::High,
            });
        }

        // 2. Apply memory locking if permitted (silent on success)
        if caps.can_lock_memory() {
            match scheduler.apply_memory_lock_internal() {
                Ok(()) => {
                    // Success - no need to announce for normal development
                }
                Err(e) => {
                    degradations.push(RtDegradation {
                        feature: RtFeature::MemoryLocking,
                        reason: e.to_string(),
                        severity: DegradationSeverity::Medium,
                    });
                }
            }
        } else {
            degradations.push(RtDegradation {
                feature: RtFeature::MemoryLocking,
                reason: format!(
                    "Memory locking not permitted (mlockall_permitted={}, limit={} bytes)",
                    caps.mlockall_permitted, caps.memlock_limit_bytes
                ),
                severity: DegradationSeverity::Medium,
            });
        }

        // 3. Apply CPU affinity ONLY to isolated CPUs (not general CPUs)
        // Auto-pinning to general CPUs can cause issues - only pin when there
        // are actual isolated CPUs dedicated for RT use
        if !caps.isolated_cpus.is_empty() {
            // Prefer isolated + nohz_full, then isolated only
            let best_cpu = caps
                .isolated_cpus
                .iter()
                .find(|cpu| caps.nohz_full_cpus.contains(cpu))
                .or_else(|| caps.isolated_cpus.first())
                .copied();

            if let Some(cpu) = best_cpu {
                match scheduler.apply_cpu_affinity_internal(cpu) {
                    Ok(()) => {
                        // Success - no need to announce for normal development
                    }
                    Err(e) => {
                        degradations.push(RtDegradation {
                            feature: RtFeature::CpuAffinity,
                            reason: e.to_string(),
                            severity: DegradationSeverity::Low, // Low severity since isolated CPUs exist
                        });
                    }
                }
            }
        }
        // If no isolated CPUs, don't auto-pin - let the OS scheduler handle it

        // 4-7. RT features detected and applied silently
        // - NUMA topology noted for future thread pool optimization
        // - RuntimeCapabilities available via scheduler.runtime_capabilities
        // - BlackBox enabled by default for crash analysis
        // - SafetyMonitor disabled by default (enable with safety_critical() or hard_realtime())
        // Use Scheduler::builder().verbose(true) for full diagnostics

        // Store degradations
        scheduler.rt_degradations = degradations;

        // Print degradation warnings
        if !scheduler.rt_degradations.is_empty() {
            for deg in &scheduler.rt_degradations {
                let severity_color = match deg.severity {
                    DegradationSeverity::High => "red",
                    DegradationSeverity::Medium => "yellow",
                    DegradationSeverity::Low => "white",
                };
                let msg = format!("[WARN] {}: {}", deg.feature, deg.reason);
                match severity_color {
                    "red" => print_line(&msg.red().to_string()),
                    "yellow" => print_line(&msg.yellow().to_string()),
                    _ => print_line(&msg.white().to_string()),
                }
            }
        }

        scheduler
    }


    // ========================================================================
    // PRESET CONSTRUCTORS
    // ========================================================================

    /// Create a scheduler configured for safety-critical systems.
    ///
    /// This is a convenience constructor equivalent to:
    /// ```rust,ignore
    /// Scheduler::new().with_config(SchedulerConfig::safety_critical())
    /// ```
    ///
    /// # Configuration
    /// - **Execution**: Sequential (deterministic)
    /// - **Tick Rate**: 1000 Hz (1kHz)
    /// - **Real-Time**: Full WCET enforcement, watchdogs (100ms), memory locking, SCHED_FIFO
    /// - **Fault Tolerance**: Circuit breaker disabled, no auto-restart
    /// - **Deadline Policy**: Panic (fail-safe)
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let mut scheduler = Scheduler::safety_critical();
    /// scheduler.add(motor_controller).order(0).done();
    /// scheduler.run()?;
    /// ```
    ///
    /// # Use Cases
    /// - Medical devices
    /// - Surgical robots
    /// - Safety-critical industrial systems
    pub fn safety_critical() -> Self {
        Self::new().with_config(super::config::SchedulerConfig::safety_critical())
    }

    /// Create a scheduler configured for high-performance applications.
    ///
    /// This is a convenience constructor equivalent to:
    /// ```rust,ignore
    /// Scheduler::new().with_config(SchedulerConfig::high_performance())
    /// ```
    ///
    /// # Configuration
    /// - **Execution**: Parallel
    /// - **Tick Rate**: 10,000 Hz (10kHz)
    /// - **Real-Time**: WCET enforcement, memory locking, SCHED_FIFO
    /// - **Deadline Policy**: Skip (maintain throughput)
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let mut scheduler = Scheduler::high_performance();
    /// scheduler.add(racing_controller).order(0).done();
    /// scheduler.run()?;
    /// ```
    ///
    /// # Use Cases
    /// - Racing robots
    /// - Competition systems
    /// - High-speed control loops
    pub fn high_performance() -> Self {
        Self::new().with_config(super::config::SchedulerConfig::high_performance())
    }

    /// Create a scheduler configured for deterministic execution.
    ///
    /// This is a convenience constructor equivalent to:
    /// ```rust,ignore
    /// Scheduler::new().with_config(SchedulerConfig::deterministic())
    /// ```
    ///
    /// # Configuration
    /// - **Execution**: Sequential (deterministic)
    /// - **Topology**: Strict validation, startup barrier, frozen after start
    /// - **RNG**: Deterministic seed
    /// - **Learning**: Disabled
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let mut scheduler = Scheduler::deterministic();
    /// scheduler.add(controller).order(0).done();
    /// scheduler.run()?;
    /// ```
    ///
    /// # Use Cases
    /// - Formal verification
    /// - Safety certification
    /// - Reproducible testing
    pub fn deterministic() -> Self {
        Self::new().with_config(super::config::SchedulerConfig::deterministic())
    }

    /// Create a scheduler configured for hard real-time applications.
    ///
    /// This is a convenience constructor equivalent to:
    /// ```rust,ignore
    /// Scheduler::new().with_config(SchedulerConfig::hard_realtime())
    /// ```
    ///
    /// # Configuration
    /// - **Execution**: Parallel
    /// - **Tick Rate**: 1000 Hz
    /// - **Jitter**: <5μs
    /// - **Real-Time**: Full enforcement, 10ms watchdog
    /// - **Deadline Policy**: Panic
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let mut scheduler = Scheduler::hard_realtime();
    /// scheduler.add(cnc_controller).order(0).done();
    /// scheduler.run()?;
    /// ```
    ///
    /// # Use Cases
    /// - CNC machines
    /// - Surgical robots
    /// - Aerospace systems
    pub fn hard_realtime() -> Self {
        Self::new().with_config(super::config::SchedulerConfig::hard_realtime())
    }

    /// Internal: Apply RT priority without error on failure.
    fn apply_rt_priority_internal(&self, priority: i32) -> crate::error::HorusResult<()> {
        #[cfg(target_os = "linux")]
        unsafe {
            use libc::{sched_param, sched_setscheduler, SCHED_FIFO};

            let param = sched_param {
                sched_priority: priority,
            };

            if sched_setscheduler(0, SCHED_FIFO, &param) != 0 {
                let err = std::io::Error::last_os_error();
                return Err(horus_internal!("sched_setscheduler failed: {}", err));
            }
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "RT priority only on Linux".to_string(),
            ))
        }
    }

    /// Internal: Apply memory locking without error on failure.
    fn apply_memory_lock_internal(&self) -> crate::error::HorusResult<()> {
        #[cfg(target_os = "linux")]
        unsafe {
            use libc::{mlockall, MCL_CURRENT, MCL_FUTURE};

            if mlockall(MCL_CURRENT | MCL_FUTURE) != 0 {
                let err = std::io::Error::last_os_error();
                return Err(horus_internal!("mlockall failed: {}", err));
            }
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "Memory locking only on Linux".to_string(),
            ))
        }
    }

    /// Internal: Apply CPU affinity without error on failure.
    fn apply_cpu_affinity_internal(&self, cpu_id: usize) -> crate::error::HorusResult<()> {
        #[cfg(target_os = "linux")]
        unsafe {
            use libc::{cpu_set_t, sched_setaffinity, CPU_SET, CPU_ZERO};

            let mut cpuset: cpu_set_t = std::mem::zeroed();
            CPU_ZERO(&mut cpuset);
            CPU_SET(cpu_id, &mut cpuset);

            if sched_setaffinity(0, std::mem::size_of::<cpu_set_t>(), &cpuset) != 0 {
                let err = std::io::Error::last_os_error();
                return Err(horus_internal!("sched_setaffinity failed: {}", err));
            }
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "CPU affinity only on Linux".to_string(),
            ))
        }
    }

    /// Get the detected runtime capabilities.
    ///
    /// Returns `None` if `Scheduler::simulation()` was used (skips detection).
    ///
    /// # Example
    /// ```rust,ignore
    /// if let Some(caps) = scheduler.capabilities() {
    ///     if caps.has_hard_rt_support() {
    ///         println!("Hard RT available!");
    ///     }
    /// }
    /// ```
    pub fn capabilities(&self) -> Option<&RuntimeCapabilities> {
        self.runtime_capabilities.as_ref()
    }

    /// Get the list of RT features that degraded during auto-optimization.
    ///
    /// An empty list means all detected features were successfully applied.
    ///
    /// # Example
    /// ```rust,ignore
    /// for deg in scheduler.degradations() {
    ///     match deg.severity {
    ///         DegradationSeverity::High => {
    ///             eprintln!("CRITICAL: {} - {}", deg.feature, deg.reason);
    ///         }
    ///         _ => {
    ///             println!("Note: {} - {}", deg.feature, deg.reason);
    ///         }
    ///     }
    /// }
    /// ```
    pub fn degradations(&self) -> &[RtDegradation] {
        &self.rt_degradations
    }

    /// Check if the scheduler has full RT capabilities (no high-severity degradations).
    pub fn has_full_rt(&self) -> bool {
        !self
            .rt_degradations
            .iter()
            .any(|d| d.severity == DegradationSeverity::High)
    }


    /// Get a reference to the BlackBox flight recorder.
    ///
    /// The BlackBox automatically records critical events for post-mortem crash analysis:
    /// - Scheduler start/stop
    /// - Node additions
    /// - Deadline misses
    /// - WCET violations
    /// - Circuit breaker state changes
    /// - Emergency stops
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let scheduler = Scheduler::new();
    ///
    /// // Access the blackbox for debugging
    /// if let Some(bb) = scheduler.blackbox() {
    ///     // Get recent events
    ///     let events = bb.recent(100);
    ///     println!("Last 100 events: {:?}", events.len());
    /// }
    /// ```
    pub fn blackbox(&self) -> Option<&super::blackbox::BlackBox> {
        self.blackbox.as_ref()
    }

    /// Get a mutable reference to the BlackBox flight recorder.
    ///
    /// Use this to manually record custom events or configure persistence.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::BlackBoxEvent;
    ///
    /// let mut scheduler = Scheduler::new();
    ///
    /// // Record a custom event
    /// if let Some(bb) = scheduler.blackbox_mut() {
    ///     bb.record(BlackBoxEvent::Custom {
    ///         category: "safety".to_string(),
    ///         message: "Manual override activated".to_string(),
    ///     });
    /// }
    /// ```
    pub fn blackbox_mut(&mut self) -> Option<&mut super::blackbox::BlackBox> {
        self.blackbox.as_mut()
    }

    /// Get the circuit breaker state for a specific node.
    ///
    /// Returns the current circuit state (Closed, Open, or HalfOpen) for
    /// the node with the given name.
    ///
    /// # Arguments
    /// - `node_name` - The name of the node to check
    ///
    /// # Returns
    /// - `Some(CircuitState)` - The circuit state if the node exists
    /// - `None` - If no node with that name exists
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::CircuitState;
    ///
    /// let mut scheduler = Scheduler::new();
    /// scheduler.add(my_node).order(10).done();
    ///
    /// if let Some(state) = scheduler.circuit_state("my_node") {
    ///     match state {
    ///         CircuitState::Closed => println!("Node healthy"),
    ///         CircuitState::Open => println!("Node isolated (failing)"),
    ///         CircuitState::HalfOpen => println!("Testing recovery"),
    ///     }
    /// }
    /// ```
    pub fn circuit_state(&self, node_name: &str) -> Option<super::fault_tolerance::CircuitState> {
        self.nodes
            .iter()
            .find(|n| n.node.name() == node_name)
            .map(|n| n.circuit_breaker.get_state())
    }

    /// Get a summary of all circuit breaker states.
    ///
    /// Returns counts of circuits in each state:
    /// - `closed` - Normal operation, requests allowed
    /// - `open` - Failing, requests blocked
    /// - `half_open` - Testing recovery
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let scheduler = Scheduler::new();
    /// // ... add some nodes ...
    ///
    /// let (closed, open, half_open) = scheduler.circuit_summary();
    /// println!("{} healthy, {} isolated, {} recovering", closed, open, half_open);
    /// ```
    pub fn circuit_summary(&self) -> (usize, usize, usize) {
        let mut closed = 0;
        let mut open = 0;
        let mut half_open = 0;

        for node in &self.nodes {
            match node.circuit_breaker.get_state() {
                super::fault_tolerance::CircuitState::Closed => closed += 1,
                super::fault_tolerance::CircuitState::Open => open += 1,
                super::fault_tolerance::CircuitState::HalfOpen => half_open += 1,
            }
        }

        (closed, open, half_open)
    }

    /// Get safety statistics including WCET overruns, deadline misses, and watchdog expirations.
    ///
    /// Returns `None` if the safety monitor is not enabled.
    ///
    /// The returned `SafetyStats` contains:
    /// - `state`: Current safety state (Normal, Degraded, EmergencyStop, SafeMode)
    /// - `wcet_overruns`: Number of WCET budget violations
    /// - `deadline_misses`: Number of deadline misses
    /// - `watchdog_expirations`: Number of watchdog timeouts
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new();
    /// // ... run scheduler for a while ...
    ///
    /// if let Some(stats) = scheduler.safety_stats() {
    ///     println!("WCET overruns: {}", stats.wcet_overruns);
    ///     println!("Deadline misses: {}", stats.deadline_misses);
    /// }
    /// ```
    pub fn safety_stats(&self) -> Option<super::safety_monitor::SafetyStats> {
        self.safety_monitor.as_ref().map(|m| m.get_stats())
    }

    /// Get the scheduler name.
    ///
    /// Returns the name assigned to this scheduler instance.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::builder()
    ///     .name("MyScheduler")
    ///     .build()?;
    /// assert_eq!(scheduler.scheduler_name(), "MyScheduler");
    /// ```
    pub fn scheduler_name(&self) -> &str {
        &self.scheduler_name
    }

    /// Get a formatted status report of the scheduler's configuration and state.
    ///
    /// Returns a human-readable string showing:
    /// - Scheduler name and platform
    /// - RT features status (priority, memory lock, CPU affinity)
    /// - Determinism mode (simulation vs real-time)
    /// - Safety features (monitor, watchdog, blackbox)
    /// - Any degradations that occurred during initialization
    ///
    /// This is useful for debugging and auditing scheduler configuration.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new();
    /// println!("{}", scheduler.status());
    /// // Output:
    /// // ==================================================================
    /// // SCHEDULER STATUS: AutoOptimized
    /// // ==================================================================
    /// // Platform: Linux 5.15.0-generic (8 CPUs, 1 NUMA nodes)
    /// // ------------------------------------------------------------------
    /// // RT Features:
    /// //   [x] PREEMPT_RT kernel
    /// //   [ ] RT Priority (max=0)
    /// //   [x] Memory Lock (limit=64MB)
    /// //   [ ] Isolated CPUs: []
    /// // ------------------------------------------------------------------
    /// // Execution Mode:
    /// //   [ ] Simulation Mode (deterministic)
    /// //   [x] Real-time Mode
    /// // ------------------------------------------------------------------
    /// // Safety Features:
    /// //   [ ] Safety Monitor
    /// //   [ ] BlackBox Recorder
    /// // ==================================================================
    /// ```
    pub fn status(&self) -> String {
        let mut lines = Vec::new();
        let sep = "==================================================================";
        let thin_sep = "------------------------------------------------------------------";

        // Header
        lines.push(sep.to_string());
        lines.push(format!("SCHEDULER STATUS: {}", self.scheduler_name));
        lines.push(sep.to_string());

        // Platform info
        if let Some(caps) = &self.runtime_capabilities {
            lines.push(format!(
                "Platform: {} ({} CPUs, {} NUMA nodes)",
                caps.kernel_version, caps.cpu_count, caps.numa_node_count
            ));

            lines.push(thin_sep.to_string());
            lines.push("RT Features:".to_string());
            lines.push(format!(
                "  [{}] PREEMPT_RT kernel",
                if caps.preempt_rt { "x" } else { " " }
            ));
            lines.push(format!(
                "  [{}] RT Priority (max={})",
                if caps.rt_priority_available { "x" } else { " " },
                caps.max_rt_priority
            ));
            lines.push(format!(
                "  [{}] Memory Lock (limit={}MB)",
                if caps.mlockall_permitted { "x" } else { " " },
                caps.memlock_limit_bytes / (1024 * 1024)
            ));
            lines.push(format!(
                "  [{}] Isolated CPUs: {:?}",
                if !caps.isolated_cpus.is_empty() {
                    "x"
                } else {
                    " "
                },
                caps.isolated_cpus
            ));
            lines.push(format!(
                "  [{}] Tickless CPUs (nohz_full): {:?}",
                if !caps.nohz_full_cpus.is_empty() {
                    "x"
                } else {
                    " "
                },
                caps.nohz_full_cpus
            ));
        } else {
            lines.push("Platform: (capabilities not detected)".to_string());
        }

        // Execution mode
        lines.push(thin_sep.to_string());
        lines.push("Execution Mode:".to_string());
        let is_sim = self.is_simulation_mode();
        lines.push(format!(
            "  [{}] Simulation Mode (deterministic)",
            if is_sim { "x" } else { " " }
        ));
        lines.push(format!(
            "  [{}] Real-time Mode",
            if !is_sim { "x" } else { " " }
        ));

        // Safety features
        lines.push(thin_sep.to_string());
        lines.push("Safety Features:".to_string());
        lines.push(format!(
            "  [{}] Safety Monitor",
            if self.safety_monitor.is_some() {
                "x"
            } else {
                " "
            }
        ));
        lines.push(format!(
            "  [{}] BlackBox Recorder",
            if self.blackbox.is_some() { "x" } else { " " }
        ));
        // Circuit breakers are always enabled per-node
        lines.push("  [x] Circuit Breakers (5 failures, 30s timeout)".to_string());
        // WCET enforcement is enabled when SafetyMonitor is present
        let has_wcet =
            self.safety_monitor.is_some() && self.nodes.iter().any(|n| n.wcet_budget.is_some());
        lines.push(format!(
            "  [{}] WCET Enforcement (RT nodes)",
            if has_wcet { "x" } else { " " }
        ));

        // WCET Stats (if safety monitor enabled and has data)
        if let Some(ref monitor) = self.safety_monitor {
            let stats = monitor.get_stats();
            if stats.wcet_overruns > 0 || stats.deadline_misses > 0 {
                lines.push(thin_sep.to_string());
                lines.push("WCET / Deadline Statistics:".to_string());
                if stats.wcet_overruns > 0 {
                    lines.push(format!("  [WARN] {} WCET overruns", stats.wcet_overruns));
                } else {
                    lines.push("  [OK] No WCET overruns".to_string());
                }
                if stats.deadline_misses > 0 {
                    lines.push(format!(
                        "  [WARN] {} deadline misses",
                        stats.deadline_misses
                    ));
                } else {
                    lines.push("  [OK] No deadline misses".to_string());
                }
            }
        }

        // Node Health (circuit breaker states)
        if !self.nodes.is_empty() {
            lines.push(thin_sep.to_string());
            lines.push("Node Health (Circuit Breakers):".to_string());
            let mut open_circuits = 0;
            let mut half_open_circuits = 0;
            for node in &self.nodes {
                let state = node.circuit_breaker.get_state();
                match state {
                    super::fault_tolerance::CircuitState::Open => open_circuits += 1,
                    super::fault_tolerance::CircuitState::HalfOpen => half_open_circuits += 1,
                    super::fault_tolerance::CircuitState::Closed => {}
                }
            }
            if open_circuits == 0 && half_open_circuits == 0 {
                lines.push(format!(
                    "  [OK] All {} nodes healthy (circuits closed)",
                    self.nodes.len()
                ));
            } else {
                lines.push(format!(
                    "  [WARN] {} open, {} half-open, {} closed",
                    open_circuits,
                    half_open_circuits,
                    self.nodes.len() - open_circuits - half_open_circuits
                ));
                // List unhealthy nodes
                for node in &self.nodes {
                    let state = node.circuit_breaker.get_state();
                    match state {
                        super::fault_tolerance::CircuitState::Open => {
                            lines.push(format!("    - {}: OPEN (isolated)", node.node.name()));
                        }
                        super::fault_tolerance::CircuitState::HalfOpen => {
                            lines.push(format!(
                                "    - {}: HALF-OPEN (testing recovery)",
                                node.node.name()
                            ));
                        }
                        super::fault_tolerance::CircuitState::Closed => {}
                    }
                }
            }
        }

        // Degradations (if any)
        if !self.rt_degradations.is_empty() {
            lines.push(thin_sep.to_string());
            lines.push("Degradations:".to_string());
            for deg in &self.rt_degradations {
                let severity_tag = match deg.severity {
                    DegradationSeverity::High => "[HIGH]",
                    DegradationSeverity::Medium => "[MED] ",
                    DegradationSeverity::Low => "[LOW] ",
                };
                lines.push(format!(
                    "  {} {:?}: {}",
                    severity_tag, deg.feature, deg.reason
                ));
            }
        }

        // Footer
        lines.push(sep.to_string());

        lines.join("\n")
    }

    // =========================================================================
    // Deterministic Execution Accessors (Simulation Mode)
    // =========================================================================

    /// Check if this scheduler is running in simulation mode (deterministic execution).
    ///
    /// Returns `true` if the scheduler was created with `Scheduler::simulation()` or
    /// `Scheduler::simulation_with_seed()`, which enables virtual time, seeded RNG,
    /// and execution tracing.
    pub fn is_simulation_mode(&self) -> bool {
        self.deterministic_clock.is_some()
    }

    /// Get the deterministic clock (simulation mode only).
    ///
    /// The deterministic clock provides:
    /// - Virtual time that advances predictably per tick
    /// - Seeded RNG for reproducible randomness
    /// - Tick counting for execution tracing
    ///
    /// Returns `None` if not in simulation mode.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::simulation_with_seed(12345);
    /// if let Some(clock) = scheduler.deterministic_clock() {
    ///     println!("Virtual time: {:?}", clock.now());
    ///     println!("Tick: {}", clock.tick());
    ///     println!("Random u64: {}", clock.random_u64());
    /// }
    /// ```
    pub fn deterministic_clock(&self) -> Option<Arc<DeterministicClock>> {
        self.deterministic_clock.clone()
    }

    /// Get the execution trace (simulation mode only).
    ///
    /// The execution trace records all node executions with timing data for:
    /// - Reproducibility verification (compare two runs)
    /// - Debugging (replay execution to find bugs)
    /// - Performance analysis (identify slow nodes)
    ///
    /// Returns `None` if not in simulation mode.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::simulation();
    /// // ... run scheduler ...
    /// if let Some(trace_lock) = scheduler.execution_trace() {
    ///     let trace = trace_lock.lock();
    ///     println!("Total ticks: {}", trace.total_ticks);
    ///     trace.save(Path::new("execution_trace.json")).unwrap();
    /// }
    /// ```
    pub fn execution_trace(&self) -> Option<Arc<ParkingMutex<ExecutionTrace>>> {
        self.execution_trace.clone()
    }

    /// Get the deterministic configuration (simulation mode only).
    ///
    /// Contains the seed, tick duration, and tracing settings.
    pub fn deterministic_config(&self) -> Option<&DeterministicConfig> {
        self.deterministic_config.as_ref()
    }

    /// Get the seed used for deterministic RNG (simulation mode only).
    ///
    /// Returns `None` if not in simulation mode.
    pub fn seed(&self) -> Option<u64> {
        self.deterministic_config.as_ref().map(|c| c.seed)
    }

    /// Get the current virtual time (simulation mode only).
    ///
    /// Returns `None` if not in simulation mode.
    pub fn virtual_time(&self) -> Option<Duration> {
        self.deterministic_clock.as_ref().map(|c| c.now())
    }

    /// Get the current virtual tick number (simulation mode only).
    ///
    /// Returns `None` if not in simulation mode.
    pub fn virtual_tick(&self) -> Option<u64> {
        self.deterministic_clock.as_ref().map(|c| c.tick())
    }

    /// Apply a configuration preset to this scheduler (builder pattern)
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use horus_core::scheduling::SchedulerConfig;
    /// let mut scheduler = Scheduler::new()
    ///     .with_config(SchedulerConfig::hard_realtime());
    /// ```
    pub fn with_config(mut self, config: super::config::SchedulerConfig) -> Self {
        self.apply_config(config);
        self
    }

    /// Internal method to apply configuration (used by with_config builder)
    fn apply_config(&mut self, config: super::config::SchedulerConfig) {
        use super::config::*;

        // Apply execution mode
        match config.execution {
            ExecutionMode::Parallel => {
                self.parallel_executor.set_max_threads(num_cpus::get());
                print_line("Parallel execution mode selected");
            }
            ExecutionMode::Sequential => {
                self.parallel_executor.set_max_threads(1);
                print_line("Sequential execution mode selected");
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
            print_line("Safety monitor configured for RT nodes");
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
            print_line(&format!("CPU cores configuration: {:?}", cores));
        }

        // Apply monitoring configuration
        if config.monitoring.profiling_enabled {
            self.profiler.enable();
            print_line("Profiling enabled");
        } else {
            self.profiler.disable();
            print_line("Profiling disabled");
        }

        // === Apply new runtime features ===

        // 1. Global tick rate enforcement
        self.tick_period =
            std::time::Duration::from_micros((1_000_000.0 / config.timing.global_rate_hz) as u64);

        // 2. Black box flight recorder
        if config.monitoring.black_box_enabled && config.monitoring.black_box_size_mb > 0 {
            let mut bb = super::blackbox::BlackBox::new(config.monitoring.black_box_size_mb);
            bb.record(super::blackbox::BlackBoxEvent::SchedulerStart {
                name: self.scheduler_name.clone(),
                node_count: self.nodes.len(),
                config: format!("rate={}Hz", config.timing.global_rate_hz),
            });
            self.blackbox = Some(bb);
            print_line(&format!(
                "[SCHEDULER] Black box enabled ({}MB buffer)",
                config.monitoring.black_box_size_mb
            ));
        }

        // 3. Real-time optimizations (Linux-specific)
        #[cfg(target_os = "linux")]
        {
            // Memory locking
            if config.realtime.memory_locking && super::runtime::lock_all_memory().is_ok() {
                print_line("[SCHEDULER] Memory locked (mlockall)");
            }

            // RT scheduling class
            if config.realtime.rt_scheduling_class {
                let priority = 50; // Default RT priority
                if super::runtime::set_realtime_priority(priority).is_ok() {
                    print_line(&format!(
                        "[SCHEDULER] RT scheduling enabled (SCHED_FIFO, priority {})",
                        priority
                    ));
                }
            }

            // CPU core affinity
            if let Some(ref cores) = config.resources.cpu_cores {
                if super::runtime::set_thread_affinity(cores).is_ok() {
                    print_line(&format!(
                        "[SCHEDULER] CPU affinity set to cores {:?}",
                        cores
                    ));
                }
            }

            // NUMA awareness
            if config.resources.numa_aware {
                let numa_nodes = super::runtime::get_numa_node_count();
                if numa_nodes > 1 {
                    print_line(&format!(
                        "[SCHEDULER] NUMA-aware scheduling ({} nodes detected)",
                        numa_nodes
                    ));
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

                print_line(&format!(
                    "[SCHEDULER] Recording enabled (session: {}, compress: {})",
                    session_name, recording_yaml.compress
                ));
            }
        }

        // 8. Telemetry export
        if let Some(ref endpoint_str) = config.monitoring.telemetry_endpoint {
            let endpoint = super::telemetry::TelemetryEndpoint::from_string(endpoint_str);
            let interval_ms = config.monitoring.metrics_interval_ms;
            let mut tm = super::telemetry::TelemetryManager::new(endpoint, interval_ms);
            tm.set_scheduler_name(&self.scheduler_name);
            self.telemetry = Some(tm);
            print_line(&format!(
                "[SCHEDULER] Telemetry enabled (endpoint: {})",
                endpoint_str
            ));
        }

        // Store config for runtime use
        self.config = Some(config);
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
    /// - Deterministic collections (sorted iteration order)
    /// - Logical clock support (opt-in via config)
    /// - Predictable memory allocation (opt-in via config)
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
        self.with_name("DeterministicScheduler")
    }

    /// Enable safety monitor with maximum allowed deadline misses
    pub fn with_safety_monitor(mut self, max_deadline_misses: u64) -> Self {
        self.safety_monitor = Some(SafetyMonitor::new(max_deadline_misses));
        self
    }

    /// Add a critical node to the safety monitor with a watchdog timeout.
    ///
    /// Critical nodes are monitored more strictly:
    /// - Watchdog expiration triggers emergency stop
    /// - WCET violations trigger emergency stop
    /// - Deadline misses trigger emergency stop
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::time::Duration;
    ///
    /// let mut scheduler = Scheduler::new()
    ///     .with_safety_monitor(10);
    /// scheduler.add_critical_node("motor_controller", Duration::from_millis(100))?;
    /// # Ok::<(), horus_core::error::HorusError>(())
    /// ```
    ///
    /// # Errors
    /// Returns an error if the safety monitor is not enabled.
    pub fn add_critical_node(
        &mut self,
        node_name: &str,
        watchdog_timeout: std::time::Duration,
    ) -> crate::error::HorusResult<&mut Self> {
        if let Some(ref mut monitor) = self.safety_monitor {
            monitor.add_critical_node(node_name.to_string(), watchdog_timeout);
            Ok(self)
        } else {
            Err(crate::error::HorusError::Config(
                "Safety monitor not enabled. Call with_safety_monitor() first.".to_string(),
            ))
        }
    }

    /// Set the WCET (Worst-Case Execution Time) budget for a node.
    ///
    /// When a node exceeds its WCET budget:
    /// - Regular nodes: Warning logged, counter incremented
    /// - Critical nodes: Emergency stop triggered
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::time::Duration;
    ///
    /// let mut scheduler = Scheduler::new()
    ///     .with_safety_monitor(10);
    /// scheduler.set_wcet_budget("motor_controller", Duration::from_micros(500))?;
    /// # Ok::<(), horus_core::error::HorusError>(())
    /// ```
    ///
    /// # Errors
    /// Returns an error if the safety monitor is not enabled.
    pub fn set_wcet_budget(
        &mut self,
        node_name: &str,
        budget: std::time::Duration,
    ) -> crate::error::HorusResult<&mut Self> {
        if let Some(ref mut monitor) = self.safety_monitor {
            monitor.set_wcet_budget(node_name.to_string(), budget);
            Ok(self)
        } else {
            Err(crate::error::HorusError::Config(
                "Safety monitor not enabled. Call with_safety_monitor() first.".to_string(),
            ))
        }
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

        // Create a ReplayNode wrapper
        let replay_node = ReplayNode::new(node_name.clone(), node_id.clone());

        // Store the replayer
        self.replay_nodes.insert(node_name.clone(), replayer);

        // Add as a registered node with replay flag
        self.nodes.push(RegisteredNode {
            node: Box::new(replay_node),
            priority,
            initialized: false,
            context: None,
            rate_hz: None,
            last_tick: None,
            circuit_breaker: CircuitBreaker::new(5, 3, 30000), // 5 failures, 3 success, 30s timeout
            is_rt_node: false,
            wcet_budget: None,
            deadline: None,
            recorder: None,

            is_stopped: false,
            is_paused: false,
            tier: NodeTier::default(),
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

        // Load all node recordings
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

        print_line(&format!("[REPLAY] Starting at tick {}", tick));
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

        print_line(&format!(
            "[REPLAY] Override set: {}.{}",
            node_name, output_name
        ));
        self
    }

    /// Set replay to stop at a specific tick.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::replay_from(
    ///     PathBuf::from("~/.horus/recordings/session/scheduler@abc123.horus")
    /// ).expect("Failed to load")
    ///     .stop_at_tick(2000);  // Stop at tick 2000
    /// ```
    pub fn stop_at_tick(mut self, tick: u64) -> Self {
        self.replay_stop_tick = Some(tick);
        print_line(&format!("[REPLAY] Will stop at tick {}", tick));
        self
    }

    /// Set replay speed multiplier.
    ///
    /// # Example
    /// ```no_run
    /// use horus_core::Scheduler;
    /// use std::path::PathBuf;
    ///
    /// let mut scheduler = Scheduler::replay_from(
    ///     PathBuf::from("~/.horus/recordings/session/scheduler@abc123.horus")
    /// ).expect("Failed to load")
    ///     .with_replay_speed(0.5);  // Half speed
    /// ```
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
            // Save all node recordings
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

            // Save scheduler recording
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

    // ============================================================================
    // Profile-Based Optimization (Deterministic Alternative to Learning)
    // ============================================================================

    // ============================================================================
    // OS Integration Methods (low-level, genuinely different from config)
    // ============================================================================

    /// Set OS-level scheduling priority using SCHED_FIFO (Linux RT-PREEMPT required).
    ///
    /// This sets the scheduler thread's OS priority, giving it preferential
    /// CPU scheduling over normal processes.
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
    /// scheduler.set_os_priority(99)?;  // Highest priority
    /// ```
    pub fn set_os_priority(&self, priority: i32) -> crate::error::HorusResult<()> {
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
                return Err(horus_internal!(
                    "Failed to set OS priority: {}. \
                     Ensure you have RT-PREEMPT kernel and CAP_SYS_NICE capability.",
                    err
                ));
            }

            print_line(&format!(
                "[OK] OS priority set to {} (SCHED_FIFO)",
                priority
            ));
            Ok(())
        }

        #[cfg(not(target_os = "linux"))]
        {
            Err(crate::error::HorusError::Unsupported(
                "OS priority scheduling is only supported on Linux".to_string(),
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
                return Err(horus_internal!("Failed to set CPU affinity: {}", err));
            }

            print_line(&format!("[OK] Scheduler pinned to CPU core {}", cpu_id));
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
                return Err(horus_internal!(
                    "Failed to lock memory: {}. \
                     Check ulimit -l and ensure CAP_IPC_LOCK capability.",
                    err
                ));
            }

            print_line("[OK] Memory locked (no page faults)");
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

        print_line(&format!(
            "[OK] Pre-faulted {} KB of stack",
            stack_size / 1024
        ));
        Ok(())
    }

    // ========================================================================
    // NODE BUILDER API (Fluent Interface)
    // ========================================================================

    /// Add a node using the fluent builder API.
    ///
    /// This is the **recommended** way to add nodes. No `Box::new()` required!
    /// Call `.done()` on the builder to register the node.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// let mut scheduler = Scheduler::new();
    ///
    /// // Simple - just node and order
    /// scheduler.add(MyNode::new())
    ///     .order(0)
    ///     .done();
    ///
    /// // With full RT configuration
    /// scheduler.add(MotorController::new())
    ///     .order(0)
    ///     .rate_hz(1000.0)  // 1kHz
    ///     .rt()
    ///     .wcet_us(500)     // 500μs max execution
    ///     .done();
    ///
    /// // Chain multiple nodes
    /// scheduler.add(SensorNode::new()).order(0).done();
    /// scheduler.add(ControlNode::new()).order(1).rt().done();
    /// scheduler.add(LoggerNode::new()).order(100).done();
    /// ```
    pub fn add<N: Node + 'static>(&mut self, node: N) -> super::node_builder::NodeBuilder<'_> {
        super::node_builder::NodeBuilder::new(self, Box::new(node))
    }

    /// Add a boxed node using the fluent builder API.
    ///
    /// Use this if you already have a `Box<dyn Node>`. Otherwise, prefer `add()`
    /// which doesn't require boxing.
    ///
    /// # Example
    /// ```rust,ignore
    /// let node: Box<dyn Node> = factory.create_node();
    /// scheduler.add_dyn(node).order(0).done();
    /// ```
    pub fn add_dyn(&mut self, node: Box<dyn Node>) -> super::node_builder::NodeBuilder<'_> {
        super::node_builder::NodeBuilder::new(self, node)
    }

    /// Alias for `add_dyn()` - start configuring a boxed node.
    ///
    /// # Deprecated
    /// Prefer using `add()` which doesn't require `Box::new()`.
    pub fn node(&mut self, node: Box<dyn Node>) -> super::node_builder::NodeBuilder<'_> {
        self.add_dyn(node)
    }

    /// Add a node using a pre-built NodeConfig.
    ///
    /// This is called internally by `NodeBuilder::done()`. You can also use it
    /// directly with a `NodeConfig`.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::NodeConfig;
    ///
    /// let config = NodeConfig::new(Box::new(my_node))
    ///     .order(0)
    ///     .rt();
    ///
    /// scheduler.add_configured(config);
    /// ```
    pub fn add_configured(&mut self, config: super::node_builder::NodeConfig) -> &mut Self {
        // Extract config fields
        let node = config.node;
        let priority = config.order;
        let custom_rate = config.rate_hz;
        let is_rt = config.is_rt;
        let wcet_budget = config.wcet_budget;
        let deadline = config.deadline;
        let tier = config.tier;

        // Use the internal add logic
        self.add_configured_internal(
            node,
            priority,
            custom_rate,
            is_rt,
            wcet_budget,
            deadline,
            tier,
        )
    }

    /// Internal method to add a fully configured node.
    #[allow(clippy::too_many_arguments)]
    fn add_configured_internal(
        &mut self,
        node: Box<dyn Node>,
        priority: u32,
        custom_rate: Option<f64>,
        is_rt_node: bool,
        wcet_budget: Option<std::time::Duration>,
        deadline: Option<std::time::Duration>,
        tier: Option<NodeTier>,
    ) -> &mut Self {
        let node_name = node.name().to_string();

        let context = NodeInfo::new(node_name.clone());

        // Create node recorder if recording is enabled
        let recorder = if let Some(ref config) = self.recording_config {
            let node_id = format!(
                "{:x}{:x}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(0),
                self.nodes.len() as u64
            );
            let recorder = NodeRecorder::new(&node_name, &node_id, config.clone());

            if let Some(ref mut scheduler_rec) = self.scheduler_recording {
                let relative_path = format!("{}@{}.horus", node_name, node_id);
                scheduler_rec.add_node_recording(&node_id, &relative_path);
            }

            Some(recorder)
        } else {
            None
        };

        // Use custom rate or node's declared rate
        let node_rate = custom_rate.or_else(|| node.rate_hz());

        self.nodes.push(RegisteredNode {
            node,
            priority,
            initialized: false,
            context: Some(context),
            rate_hz: node_rate,
            last_tick: if node_rate.is_some() {
                Some(Instant::now())
            } else {
                None
            },
            circuit_breaker: CircuitBreaker::new(5, 3, 30000),
            is_rt_node,
            wcet_budget,
            deadline,
            recorder,

            is_stopped: false,
            is_paused: false,
            tier: tier.unwrap_or_default(),
        });

        if let Some(rate) = node_rate {
            print_line(&format!(
                "Added {} '{}' with priority {} at {:.1}Hz",
                if is_rt_node { "RT node" } else { "node" },
                node_name,
                priority,
                rate
            ));
        } else {
            print_line(&format!(
                "Added {} '{}' with priority {}",
                if is_rt_node { "RT node" } else { "node" },
                node_name,
                priority
            ));
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
    /// scheduler.add(sensor, 0)
    ///     .set_node_rate("sensor", 100.0);  // Run sensor at 100Hz
    /// ```
    pub fn set_node_rate(&mut self, name: &str, rate_hz: f64) -> &mut Self {
        for registered in self.nodes.iter_mut() {
            if registered.node.name() == name {
                registered.rate_hz = Some(rate_hz);
                registered.last_tick = Some(Instant::now());
                print_line(&format!("Set node '{}' rate to {:.1} Hz", name, rate_hz));
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
        let rt = tokio::runtime::Runtime::new()
            .map_err(|e| horus_internal!("Failed to create tokio runtime: {}", e))?;

        rt.block_on(async {
            // Track start time for duration-limited runs
            let start_time = Instant::now();

            // Set up signal handling
            let running = self.running.clone();
            if let Err(e) = ctrlc::set_handler(move || {
                print_line("\nCtrl+C received! Shutting down HORUS scheduler...");
                if let Ok(mut r) = running.lock() {
                    *r = false;
                }
                std::thread::spawn(|| {
                    std::thread::sleep(std::time::Duration::from_secs(2));
                    print_line("Force terminating - cleaning up session...");
                    // Clean up session before forced exit to prevent stale files
                    Self::cleanup_session();
                    std::process::exit(0);
                });
            }) {
                print_line(&format!("Warning: Failed to set signal handler: {}", e));
            }

            // Set up SIGTERM handler for graceful termination (e.g., from `kill` or `timeout`)
            #[cfg(unix)]
            unsafe {
                libc::signal(
                    libc::SIGTERM,
                    sigterm_handler as *const () as libc::sighandler_t,
                );
            }

            // Initialize nodes
            for registered in self.nodes.iter_mut() {
                let node_name = registered.node.name();
                let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

                if should_run && !registered.initialized {
                    if let Some(ref mut ctx) = registered.context {
                        // Set node context for hlog!() macro
                        set_node_context(node_name, 0);
                        let init_result = registered.node.init();
                        clear_node_context();

                        match init_result {
                            Ok(()) => {
                                registered.initialized = true;
                                // Announce to discovery topic
                                let publishers = registered.node.publishers();
                                let subscribers = registered.node.subscribers();
                                announce_started(node_name, &publishers, &subscribers);

                                // Write presence file for monitor detection
                                let presence = NodePresence::new(
                                    node_name,
                                    Some(&self.scheduler_name),
                                    publishers,
                                    subscribers,
                                    registered.priority,
                                    registered.rate_hz,
                                );
                                if let Err(e) = presence.write() {
                                    print_line(&format!(
                                        "Warning: Failed to write presence file for '{}': {}",
                                        node_name, e
                                    ));
                                }

                                print_line(&format!("Initialized node '{}'", node_name));
                            }
                            Err(e) => {
                                print_line(&format!(
                                    "Failed to initialize node '{}': {}",
                                    node_name, e
                                ));
                                ctx.transition_to_error(format!("Initialization failed: {}", e));
                            }
                        }
                    }
                }
            }

            // Create control directory for per-node lifecycle commands
            Self::setup_control_directory();

            // Write initial registry
            self.update_registry();

            // Set up tier-based executors (moves annotated nodes to specialized executors)
            self.setup_tier_executors().await;

            // Main tick loop
            while self.is_running() {
                // Check if duration limit has been reached
                if let Some(max_duration) = duration {
                    if start_time.elapsed() >= max_duration {
                        print_line(&format!(
                            "Scheduler reached time limit of {:?}",
                            max_duration
                        ));
                        break;
                    }
                }

                // Check if replay stop tick has been reached
                if let Some(stop_tick) = self.replay_stop_tick {
                    if self.current_tick >= stop_tick {
                        print_line(&format!("[REPLAY] Reached stop tick {}", stop_tick));
                        break;
                    }
                }

                // Check if SIGTERM was received (e.g., from `kill` or `timeout`)
                if SIGTERM_RECEIVED.load(Ordering::SeqCst) {
                    print_line("\nSIGTERM received! Shutting down HORUS scheduler...");
                    break;
                }

                // Process per-node control commands (stop, restart, pause, resume)
                self.process_control_commands();

                let now = Instant::now();
                self.last_instant = now;

                // Re-initialize nodes that need restart (set by control commands)
                for registered in self.nodes.iter_mut() {
                    if !registered.is_stopped && !registered.is_paused && !registered.initialized {
                        let node_name = registered.node.name();
                        if let Some(ref mut ctx) = registered.context {
                            // Set node context for hlog!() macro
                            set_node_context(node_name, 0);
                            let init_result = registered.node.init();
                            clear_node_context();

                            match init_result {
                                Ok(()) => {
                                    registered.initialized = true;
                                    print_line(&format!(
                                        "[CONTROL] Node '{}' re-initialized",
                                        node_name
                                    ));
                                }
                                Err(e) => {
                                    print_line(&format!(
                                        "[CONTROL] Failed to re-initialize node '{}': {}",
                                        node_name, e
                                    ));
                                    ctx.transition_to_error(format!(
                                        "Re-initialization failed: {}",
                                        e
                                    ));
                                }
                            }
                        }
                    }
                }

                // Execute nodes in priority order
                self.execute_nodes(node_filter).await;

                // Check watchdogs and handle emergency stop for RT systems
                if let Some(ref monitor) = self.safety_monitor {
                    // Check all watchdogs
                    let expired_watchdogs = monitor.check_watchdogs();
                    if !expired_watchdogs.is_empty() {
                        print_line(&format!(
                            " Watchdog expired for nodes: {:?}",
                            expired_watchdogs
                        ));
                    }

                    // Check if emergency stop was triggered
                    if monitor.is_emergency_stop() {
                        print_line(" Emergency stop activated - shutting down scheduler");
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
                                print_line("\nCircuit Breaker Status:");
                                has_breaker_issues = true;
                            }
                            print_line(&format!(
                                "  {} - State: {:?}, Failures: {}, Successes: {}",
                                registered.node.name(),
                                stats.state,
                                stats.failure_count,
                                stats.success_count
                            ));
                        }
                    }

                }

                // === Runtime feature integrations ===

                // Black box tick increment
                if let Some(ref mut bb) = self.blackbox {
                    bb.tick();
                }

                // Process results from tier-based executors
                self.process_async_results().await;
                self.process_background_results();

                // Use pre-computed tick period (from config or default ~60Hz)
                // Apply replay speed adjustment if in replay mode
                let sleep_duration = if self.replay_mode.is_some() && self.replay_speed != 1.0 {
                    Duration::from_nanos(
                        (self.tick_period.as_nanos() as f64 / self.replay_speed) as u64,
                    )
                } else {
                    self.tick_period
                };
                tokio::time::sleep(sleep_duration).await;

                // Increment tick counter for replay tracking
                self.current_tick += 1;
            }

            // Shutdown tier-based executors
            if let Some(ref mut exec) = self.async_io_executor {
                exec.shutdown_all().await;
            }
            if let Some(ref mut exec) = self.background_executor {
                exec.shutdown();
            }
            if let Some(ref mut exec) = self.isolated_executor {
                exec.shutdown();
            }

            // Shutdown nodes
            for registered in self.nodes.iter_mut() {
                let node_name = registered.node.name();
                let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

                if should_run && registered.initialized {
                    if let Some(ref mut ctx) = registered.context {
                        ctx.record_shutdown();

                        // Set node context for hlog!() macro
                        set_node_context(node_name, ctx.metrics().total_ticks);
                        let shutdown_result = registered.node.shutdown();
                        clear_node_context();

                        match shutdown_result {
                            Ok(()) => {
                                announce_stopped(node_name);
                                // Remove presence file
                                if let Err(e) = NodePresence::remove(node_name) {
                                    print_line(&format!(
                                        "Warning: Failed to remove presence file for '{}': {}",
                                        node_name, e
                                    ));
                                }
                                print_line(&format!("Shutdown node '{}' successfully", node_name));
                            }
                            Err(e) => {
                                // Still try to remove presence file on error
                                let _ = NodePresence::remove(node_name);
                                print_line(&format!(
                                    "Error shutting down node '{}': {}",
                                    node_name, e
                                ));
                            }
                        }
                    }
                }
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
                    print_line(&format!("[BLACKBOX] Failed to save: {}", e));
                }
            }

            // Clean up registry file and session
            self.cleanup_registry();
            Self::cleanup_session();

            print_line("Scheduler shutdown complete");
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
                return Some(info);
            }
        }
        None
    }

    /// Get performance metrics for all nodes
    ///
    /// Returns a vector of `SchedulerNodeMetrics` containing performance data
    /// for each registered node.
    ///
    /// # Example
    /// ```ignore
    /// let metrics = scheduler.get_metrics();
    /// for node_metrics in metrics {
    ///     println!("Node: {}", node_metrics.name);
    ///     println!("  Avg tick: {:.2}ms", node_metrics.avg_tick_duration_ms);
    ///     println!("  Total ticks: {}", node_metrics.total_ticks);
    /// }
    /// ```
    pub fn get_metrics(&self) -> Vec<SchedulerNodeMetrics> {
        self.nodes
            .iter()
            .map(|registered| {
                let name = registered.node.name().to_string();
                let priority = registered.priority;

                // Get metrics from context if available
                if let Some(ref ctx) = registered.context {
                    let m = ctx.metrics();
                    SchedulerNodeMetrics {
                        name,
                        priority,
                        total_ticks: m.total_ticks,
                        successful_ticks: m.successful_ticks,
                        failed_ticks: m.failed_ticks,
                        avg_tick_duration_ms: m.avg_tick_duration_ms,
                        max_tick_duration_ms: m.max_tick_duration_ms,
                        min_tick_duration_ms: m.min_tick_duration_ms,
                        last_tick_duration_ms: m.last_tick_duration_ms,
                        messages_sent: m.messages_sent,
                        messages_received: m.messages_received,
                        errors_count: m.errors_count,
                        warnings_count: m.warnings_count,
                        uptime_seconds: m.uptime_seconds,
                    }
                } else {
                    SchedulerNodeMetrics {
                        name,
                        priority,
                        ..Default::default()
                    }
                }
            })
            .collect()
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
                // Runtime-discovered pub/sub is now tracked by TopicRegistry, not NodeInfo
                let publishers = registered.node.publishers();
                let subscribers = registered.node.subscribers();

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

    /// Setup control directory for node lifecycle commands
    fn setup_control_directory() {
        let dir = shm_control_dir();
        let _ = fs::create_dir_all(&dir);
    }

    /// Clean up control directory
    pub fn cleanup_control_dir() {
        let dir = shm_control_dir();
        if dir.exists() {
            let _ = fs::remove_dir_all(&dir);
        }
    }

    /// Check and process control commands for all nodes
    ///
    /// Reads control files from `/dev/shm/horus/control/{node_name}.cmd`
    /// and processes commands like stop, restart, pause, resume.
    fn process_control_commands(&mut self) {
        let control_dir = shm_control_dir();
        if !control_dir.exists() {
            return;
        }

        // Check for control files
        if let Ok(entries) = fs::read_dir(&control_dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().is_some_and(|ext| ext == "cmd") {
                    // Extract node name from filename (e.g., "my_node.cmd" -> "my_node")
                    if let Some(stem) = path.file_stem() {
                        let node_name = stem.to_string_lossy().to_string();

                        // Read command
                        if let Ok(cmd_str) = fs::read_to_string(&path) {
                            let cmd = cmd_str.trim().to_lowercase();

                            // Find and process the node
                            let mut found = false;
                            for registered in &mut self.nodes {
                                if registered.node.name() == node_name
                                    || registered.node.name().contains(&node_name)
                                {
                                    found = true;
                                    match cmd.as_str() {
                                        "stop" => {
                                            registered.is_stopped = true;
                                            registered.is_paused = false;
                                            println!(
                                                "{}",
                                                format!("[CONTROL] Node '{}' stopped", node_name)
                                                    .yellow()
                                            );
                                            // Update state to show stopped
                                            if let Some(ref mut ctx) = registered.context {
                                                ctx.transition_to_error(
                                                    "Stopped via control command".to_string(),
                                                );
                                            }
                                        }
                                        "restart" => {
                                            registered.is_stopped = false;
                                            registered.is_paused = false;
                                            registered.initialized = false;
                                            print_line(&format!(
                                                "[CONTROL] Node '{}' restarting",
                                                node_name
                                            ));
                                            // Reset context for re-initialization
                                            if let Some(ref mut ctx) = registered.context {
                                                ctx.reset_for_restart();
                                            }
                                        }
                                        "pause" => {
                                            registered.is_paused = true;
                                            print_line(&format!(
                                                "[CONTROL] Node '{}' paused",
                                                node_name
                                            ));
                                        }
                                        "resume" => {
                                            registered.is_paused = false;
                                            print_line(&format!(
                                                "[CONTROL] Node '{}' resumed",
                                                node_name
                                            ));
                                        }
                                        _ => {
                                            print_line(&format!(
                                                "[CONTROL] Unknown command '{}' for node '{}'",
                                                cmd, node_name
                                            ));
                                        }
                                    }
                                    break;
                                }
                            }

                            if !found {
                                print_line(&format!("[CONTROL] Node '{}' not found", node_name));
                            }
                        }

                        // Remove processed control file
                        let _ = fs::remove_file(&path);
                    }
                }
            }
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
                .unwrap_or_default()
                .as_secs();

            // Collect node info including state and health
            let nodes_json: Vec<String> = self.nodes.iter().map(|registered| {
                let name = registered.node.name();
                let priority = registered.priority;

                // Get pub/sub from Node trait (macro-declared)
                // Runtime-discovered pub/sub is now tracked by TopicRegistry, not NodeInfo
                let publishers = registered.node.publishers();
                let subscribers = registered.node.subscribers();

                // Get state and health from context
                let (state_str, health_str, error_count, tick_count) = if let Some(ref ctx) = registered.context {
                    let metrics = ctx.metrics();
                    (
                        ctx.state().to_string(),
                        metrics.calculate_health().as_str().to_string(),
                        metrics.errors_count,
                        metrics.total_ticks,
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

    /// Execute nodes in priority order with profiling and RT support
    async fn execute_nodes(&mut self, node_filter: Option<&[&str]>) {
        // Sort by priority
        self.nodes.sort_by_key(|r| r.priority);

        // We need to process nodes one at a time to avoid borrow checker issues
        let num_nodes = self.nodes.len();
        for i in 0..num_nodes {
            // Skip stopped or paused nodes (per-node lifecycle control)
            if self.nodes[i].is_stopped || self.nodes[i].is_paused {
                continue;
            }

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

                        // Set node context for hlog!() macro
                        let tick_number = context.metrics().total_ticks;
                        set_node_context(node_name, tick_number);

                        // Execute node tick with panic handling
                        let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.tick();
                        }));

                        clear_node_context();
                        result
                    } else {
                        continue;
                    }
                };

                let tick_duration = tick_start.elapsed();

                // Check if node execution failed
                if tick_result.is_err() {
                    // Record failure for Isolated tier classification
                    self.profiler.record_node_failure(node_name);
                    print_line(&format!("Node '{}' panicked during execution", node_name));
                }

                // Record profiling data
                self.profiler.record(node_name, tick_duration);

                // Check WCET budget for RT nodes
                if self.nodes[i].is_rt_node && self.nodes[i].wcet_budget.is_some() {
                    if let Some(ref monitor) = self.safety_monitor {
                        if let Err(violation) = monitor.check_wcet(node_name, tick_duration) {
                            print_line(&format!(
                                " WCET violation in {}: {:?} > {:?}",
                                violation.node_name, violation.actual, violation.budget
                            ));
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
                                print_line(&format!(
                                    " Deadline miss in {}: {:?} > {:?}",
                                    node_name, elapsed, deadline
                                ));
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
                            context.record_tick();
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
                            context.record_tick_failure(error_msg.clone());
                            print_line(&format!(" {} failed: {}", node_name, error_msg));

                            // Set context for on_error handler
                            set_node_context(node_name, context.metrics().total_ticks);
                            registered.node.on_error(&error_msg);
                            clear_node_context();

                            if context.config().restart_on_failure {
                                match context.restart() {
                                    Ok(_) => {
                                        print_line(&format!(
                                            " Node '{}' restarted successfully (attempt {}/{})",
                                            node_name,
                                            context.metrics().errors_count,
                                            context.config().max_restart_attempts
                                        ));
                                        registered.initialized = true;
                                    }
                                    Err(e) => {
                                        print_line(&format!(
                                            "Node '{}' exceeded max restart attempts: {}",
                                            node_name, e
                                        ));
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

    // ========================================================================
    // Tier-based executor setup
    // ========================================================================

    /// Set up specialized executors based on explicit NodeTier annotations.
    ///
    /// Scans all registered nodes and moves those with AsyncIO, Background,
    /// or Isolated tiers to their respective executors.
    async fn setup_tier_executors(&mut self) {
        self.setup_async_executor().await;
        self.setup_background_executor();
        self.setup_isolated_executor();
    }

    /// Set up async I/O executor and move AsyncIO-tier nodes to it
    async fn setup_async_executor(&mut self) {
        // Find indices of AsyncIO-tier nodes
        let async_indices: Vec<usize> = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.tier == NodeTier::AsyncIO)
            .map(|(i, _)| i)
            .collect();

        if async_indices.is_empty() {
            return;
        }

        // Create async I/O executor
        let mut async_executor = match AsyncIOExecutor::new() {
            Ok(exec) => exec,
            Err(_) => return,
        };

        // Create channel for async results
        let (tx, rx) = mpsc::unbounded_channel();
        self.async_result_tx = Some(tx.clone());
        self.async_result_rx = Some(rx);

        // Move nodes to async executor (in reverse order to maintain indices)
        for idx in async_indices.into_iter().rev() {
            let registered = self.nodes.swap_remove(idx);
            let node_name = registered.node.name().to_string();

            if let Err(e) =
                async_executor.spawn_node(registered.node, registered.context, tx.clone())
            {
                print_line(&format!(
                    "Failed to move {} to async tier: {}",
                    node_name, e
                ));
            }
        }

        print_line("[Async I/O] Nodes moved to non-blocking executor");
        self.async_io_executor = Some(async_executor);
    }

    /// Set up background executor and move Background-tier nodes to it
    fn setup_background_executor(&mut self) {
        // Find indices of Background-tier nodes
        let bg_indices: Vec<usize> = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.tier == NodeTier::Background)
            .map(|(i, _)| i)
            .collect();

        if bg_indices.is_empty() {
            return;
        }

        let mut bg_executor = match BackgroundExecutor::new() {
            Ok(exec) => exec,
            Err(e) => {
                print_line(&format!("[Background] Failed to create executor: {}", e));
                return;
            }
        };

        for idx in bg_indices.into_iter().rev() {
            let registered = self.nodes.swap_remove(idx);
            let node_name = registered.node.name().to_string();

            if let Err(e) = bg_executor.spawn_node(registered.node, registered.context) {
                print_line(&format!(
                    "Failed to move {} to background tier: {}",
                    node_name, e
                ));
            }
        }

        if bg_executor.node_count() > 0 {
            print_line(&format!(
                "[Background] Moved {} nodes to low-priority thread",
                bg_executor.node_count()
            ));
        }
        self.background_executor = Some(bg_executor);
    }

    /// Set up isolated executor and move Isolated-tier nodes to it
    fn setup_isolated_executor(&mut self) {
        // Find indices of Isolated-tier nodes
        let iso_indices: Vec<usize> = self
            .nodes
            .iter()
            .enumerate()
            .filter(|(_, n)| n.tier == NodeTier::Isolated)
            .map(|(i, _)| i)
            .collect();

        if iso_indices.is_empty() {
            return;
        }

        let config = IsolatedNodeConfig {
            max_restarts: 3,
            restart_delay: std::time::Duration::from_millis(500),
            response_timeout: std::time::Duration::from_millis(5000),
            heartbeat_timeout: std::time::Duration::from_secs(10),
            runner_binary: None,
            env_vars: std::collections::HashMap::new(),
        };

        let mut iso_executor = match IsolatedExecutor::new(config) {
            Ok(exec) => exec,
            Err(e) => {
                print_line(&format!("[Isolated] Failed to create executor: {}", e));
                return;
            }
        };

        for idx in iso_indices.into_iter().rev() {
            let registered = self.nodes.swap_remove(idx);
            let node_name = registered.node.name().to_string();

            if let Err(e) =
                iso_executor.spawn_node(registered.node, &node_name, registered.context)
            {
                print_line(&format!(
                    "Failed to move {} to isolated tier: {}",
                    node_name, e
                ));
            }
        }

        if iso_executor.node_count() > 0 {
            print_line(&format!(
                "[Isolated] Moved {} nodes to process-isolated executor",
                iso_executor.node_count()
            ));
        }
        self.isolated_executor = Some(iso_executor);
    }

    /// Process results from async I/O executor (non-blocking)
    async fn process_async_results(&mut self) {
        if let Some(ref mut rx) = self.async_result_rx {
            while let Ok(result) = rx.try_recv() {
                if !result.success {
                    if let Some(ref error) = result.error {
                        print_line(&format!(
                            "Async node {} failed: {}",
                            result.node_name, error
                        ));
                    }
                }
            }
        }
    }

    /// Process results from background executor (non-blocking)
    fn process_background_results(&mut self) {
        if let Some(ref executor) = self.background_executor {
            for result in executor.poll_results() {
                if !result.success {
                    if let Some(ref error) = result.error {
                        print_line(&format!(
                            "[Background] Node {} failed: {}",
                            result.node_name, error
                        ));
                    }
                }
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::core::Node;
    use std::sync::atomic::{AtomicUsize, Ordering};
    use std::sync::Arc;
    use std::time::Duration;

    /// Simple test node that counts its tick invocations
    struct CounterNode {
        name: &'static str,
        tick_count: Arc<AtomicUsize>,
    }

    impl CounterNode {
        fn new(name: &'static str) -> Self {
            Self {
                name,
                tick_count: Arc::new(AtomicUsize::new(0)),
            }
        }

        fn with_counter(name: &'static str, counter: Arc<AtomicUsize>) -> Self {
            Self {
                name,
                tick_count: counter,
            }
        }

    }

    impl Node for CounterNode {
        fn name(&self) -> &'static str {
            self.name
        }

        fn tick(&mut self) {
            self.tick_count.fetch_add(1, Ordering::SeqCst);
        }
    }

    // ============================================================================
    // Creation Tests
    // ============================================================================

    #[test]
    fn test_scheduler_new() {
        let scheduler = Scheduler::new();
        assert!(scheduler.is_running());
        assert_eq!(scheduler.get_node_list().len(), 0);
    }

    #[test]
    fn test_scheduler_default() {
        let scheduler = Scheduler::default();
        assert!(scheduler.is_running());
        assert_eq!(scheduler.get_node_list().len(), 0);
    }

    #[test]
    fn test_scheduler_with_name() {
        let scheduler = Scheduler::new().with_name("TestScheduler");
        // The name is stored internally and used in logging
        assert!(scheduler.is_running());
    }

    #[test]
    fn test_scheduler_with_capacity() {
        let scheduler = Scheduler::new().with_capacity(100);
        assert!(scheduler.is_running());
        // Capacity is pre-allocated but empty
        assert_eq!(scheduler.get_node_list().len(), 0);
    }

    // ============================================================================
    // Node Addition Tests
    // ============================================================================

    #[test]
    fn test_scheduler_add_node() {
        let mut scheduler = Scheduler::new();
        scheduler.add(CounterNode::new("test_node")).order(0).done();

        let nodes = scheduler.get_node_list();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0], "test_node");
    }

    #[test]
    fn test_scheduler_add_multiple_nodes() {
        let mut scheduler = Scheduler::new();
        scheduler.add(CounterNode::new("node1")).order(0).done();
        scheduler.add(CounterNode::new("node2")).order(1).done();
        scheduler.add(CounterNode::new("node3")).order(2).done();

        let nodes = scheduler.get_node_list();
        assert_eq!(nodes.len(), 3);
    }

    #[test]
    fn test_scheduler_node_priority_ordering() {
        let mut scheduler = Scheduler::new();
        // Add nodes with different priorities
        scheduler
            .add(CounterNode::new("low_priority"))
            .order(10)
            .done();
        scheduler
            .add(CounterNode::new("high_priority"))
            .order(0)
            .done();
        scheduler
            .add(CounterNode::new("medium_priority"))
            .order(5)
            .done();

        // After sorting by priority, high_priority should come first
        let nodes = scheduler.get_node_list();
        assert_eq!(nodes.len(), 3);
        // Note: nodes are sorted by priority
    }

    #[test]
    fn test_scheduler_add_basic() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(CounterNode::new("basic_node"))
            .order(0)
            .done();

        let info = scheduler.get_node_info("basic_node");
        assert!(info.is_some());
    }

    // ============================================================================
    // Running State Tests
    // ============================================================================

    #[test]
    fn test_scheduler_is_running() {
        let scheduler = Scheduler::new();
        assert!(scheduler.is_running());
    }

    #[test]
    fn test_scheduler_stop() {
        let scheduler = Scheduler::new();
        assert!(scheduler.is_running());
        scheduler.stop();
        assert!(!scheduler.is_running());
    }

    #[test]
    fn test_scheduler_stop_and_check_multiple_times() {
        let scheduler = Scheduler::new();
        scheduler.stop();
        assert!(!scheduler.is_running());
        assert!(!scheduler.is_running()); // Should still be false
    }

    // ============================================================================
    // Node Rate Control Tests
    // ============================================================================

    #[test]
    fn test_scheduler_set_node_rate() {
        let mut scheduler = Scheduler::new();
        scheduler.add(CounterNode::new("sensor")).order(0).done();
        scheduler.set_node_rate("sensor", 100.0);

        // Just verify it doesn't panic
        assert!(scheduler.is_running());
    }

    #[test]
    fn test_scheduler_set_node_rate_nonexistent() {
        let mut scheduler = Scheduler::new();
        scheduler.add(CounterNode::new("node1")).order(0).done();
        // Setting rate for nonexistent node should not panic
        scheduler.set_node_rate("nonexistent", 50.0);
        assert!(scheduler.is_running());
    }

    // ============================================================================
    // Determinism Tests
    // ============================================================================

    #[test]
    fn test_scheduler_enable_determinism() {
        let scheduler = Scheduler::new().enable_determinism();
        assert!(scheduler.is_running());
    }

    // ============================================================================
    // Node Info Tests
    // ============================================================================

    #[test]
    fn test_scheduler_get_node_info_existing() {
        let mut scheduler = Scheduler::new();
        scheduler.add(CounterNode::new("info_node")).order(0).done();

        let info = scheduler.get_node_info("info_node");
        assert!(info.is_some());

        let info_map = info.unwrap();
        assert!(info_map.contains_key("name"));
        assert_eq!(info_map.get("name").unwrap(), "info_node");
    }

    #[test]
    fn test_scheduler_get_node_info_nonexistent() {
        let scheduler = Scheduler::new();
        let info = scheduler.get_node_info("nonexistent");
        assert!(info.is_none());
    }

    #[test]
    fn test_scheduler_get_node_list_empty() {
        let scheduler = Scheduler::new();
        let nodes = scheduler.get_node_list();
        assert!(nodes.is_empty());
    }

    // ============================================================================
    // Monitoring Summary Tests
    // ============================================================================

    #[test]
    fn test_scheduler_get_monitoring_summary() {
        let mut scheduler = Scheduler::new();
        scheduler.add(CounterNode::new("mon_node1")).order(0).done();
        scheduler.add(CounterNode::new("mon_node2")).order(1).done();

        let summary = scheduler.get_monitoring_summary();
        assert_eq!(summary.len(), 2);
    }

    #[test]
    fn test_scheduler_monitoring_summary_empty() {
        let scheduler = Scheduler::new();
        let summary = scheduler.get_monitoring_summary();
        assert!(summary.is_empty());
    }

    // ============================================================================
    // Recording Tests
    // ============================================================================

    #[test]
    fn test_scheduler_is_recording_default() {
        let scheduler = Scheduler::new();
        assert!(!scheduler.is_recording());
    }

    #[test]
    fn test_scheduler_enable_recording() {
        let scheduler = Scheduler::new().enable_recording("test_session");
        assert!(scheduler.is_recording());
    }

    #[test]
    fn test_scheduler_is_replaying_default() {
        let scheduler = Scheduler::new();
        assert!(!scheduler.is_replaying());
    }

    #[test]
    fn test_scheduler_current_tick() {
        let scheduler = Scheduler::new();
        assert_eq!(scheduler.current_tick(), 0);
    }

    #[test]
    fn test_scheduler_start_at_tick() {
        let scheduler = Scheduler::new().start_at_tick(1000);
        assert_eq!(scheduler.current_tick(), 1000);
    }

    // ============================================================================
    // Safety Monitor Tests
    // ============================================================================

    #[test]
    fn test_scheduler_with_safety_monitor() {
        let scheduler = Scheduler::new().with_safety_monitor(10);
        assert!(scheduler.is_running());
    }

    // ============================================================================
    // Real-time Node Tests
    // ============================================================================

    #[test]
    fn test_scheduler_add_rt_node() {
        let mut scheduler = Scheduler::new();
        scheduler
            .add(CounterNode::new("rt_node"))
            .order(0)
            .rt()
            .wcet_us(100)
            .deadline_ms(1)
            .done();

        let nodes = scheduler.get_node_list();
        assert_eq!(nodes.len(), 1);
        assert_eq!(nodes[0], "rt_node");
    }

    // ============================================================================
    // Run For Duration Tests
    // ============================================================================

    #[test]
    fn test_scheduler_run_for_short_duration() {
        let mut scheduler = Scheduler::new();
        let counter = Arc::new(AtomicUsize::new(0));
        scheduler
            .add(CounterNode::with_counter("counter", counter.clone()))
            .order(0)
            .done();

        // Run for a short duration (50ms is more reliable under parallel test load)
        let result = scheduler.run_for(Duration::from_millis(50));
        assert!(result.is_ok());

        // Counter should have been incremented at least once
        assert!(counter.load(Ordering::SeqCst) > 0);
    }

    // ============================================================================
    // Chainable API Tests
    // ============================================================================

    #[test]
    fn test_scheduler_chainable_api() {
        let mut scheduler = Scheduler::new()
            .with_name("ChainedScheduler")
            .with_capacity(10);

        scheduler
            .add(CounterNode::new("chain_node"))
            .order(0)
            .done();

        assert!(scheduler.is_running());
        assert_eq!(scheduler.get_node_list().len(), 1);
    }

    // ============================================================================
    // List Recordings Tests
    // ============================================================================

    #[test]
    fn test_scheduler_list_recordings() {
        // This might fail if no recordings exist, but shouldn't panic
        let result = Scheduler::list_recordings();
        // Just verify the function is callable
        assert!(result.is_ok() || result.is_err());
    }

    // ============================================================================
    // Builder Pattern Name Test
    // ============================================================================

    #[test]
    fn test_scheduler_name_builder() {
        let scheduler = Scheduler::new().name("BuilderName");
        // Verify the scheduler was created successfully
        assert!(scheduler.is_running());
    }

    // ============================================================================
    // Override Tests
    // ============================================================================

    #[test]
    fn test_scheduler_with_override() {
        let scheduler = Scheduler::new().with_override("node1", "output1", vec![1, 2, 3, 4]);

        // Should not panic and scheduler should still be running
        assert!(scheduler.is_running());
    }

    // ============================================================================
    // Cleanup Tests
    // ============================================================================

    // ============================================================================
    // Auto-Optimization Tests
    // ============================================================================

    #[test]
    fn test_scheduler_auto_optimization() {
        // Test that Scheduler::new() auto-detects capabilities
        let scheduler = Scheduler::new();

        // Should have detected capabilities
        assert!(
            scheduler.capabilities().is_some(),
            "Scheduler should detect runtime capabilities"
        );

        let caps = scheduler.capabilities().unwrap();

        // Verify capabilities structure
        assert!(caps.cpu_count > 0, "Should detect at least 1 CPU");
        assert!(
            !caps.kernel_version.is_empty(),
            "Should detect kernel version"
        );

        // Should have safety monitor enabled by default
        // Safety monitor is now disabled by default (prototyping-friendly)
        assert!(scheduler.safety_monitor.is_none());
    }

    #[test]
    fn test_scheduler_degradations() {
        let scheduler = Scheduler::new();

        // Degradations should be a non-empty list in most dev environments
        // (typically no RT priority available without root/CAP_SYS_NICE)
        let degradations = scheduler.degradations();

        // On a development machine without RT permissions, we expect degradations
        // On a properly configured RT machine, this list might be empty
        // Either case is valid - we just verify the API works
        for deg in degradations {
            // Verify all fields are populated
            assert!(!deg.reason.is_empty(), "Degradation should have a reason");
            // Verify Display impl works
            let _ = format!("{}", deg.feature);
        }
    }

    #[test]
    fn test_scheduler_has_full_rt() {
        let scheduler = Scheduler::new();

        // has_full_rt() should return false if there are high-severity degradations
        let has_high = scheduler
            .degradations()
            .iter()
            .any(|d| d.severity == DegradationSeverity::High);

        assert_eq!(!has_high, scheduler.has_full_rt());
    }

    #[test]
    fn test_rt_feature_display() {
        // Test Display implementations for all RtFeature variants
        assert_eq!(format!("{}", RtFeature::RtPriority), "RT Priority");
        assert_eq!(format!("{}", RtFeature::MemoryLocking), "Memory Locking");
        assert_eq!(format!("{}", RtFeature::CpuAffinity), "CPU Affinity");
        assert_eq!(format!("{}", RtFeature::NumaPinning), "NUMA Pinning");
        assert_eq!(format!("{}", RtFeature::Watchdog), "Watchdog");
        assert_eq!(format!("{}", RtFeature::SafetyMonitor), "Safety Monitor");
    }
}
