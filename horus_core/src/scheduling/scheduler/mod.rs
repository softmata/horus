use crate::core::hlog::{clear_node_context, set_node_context};
use crate::core::{announce_started, announce_stopped, DurationExt, Node, NodeInfo, NodePresence};
use crate::error::{HorusContext, HorusResult};
use crate::memory::platform::shm_control_dir;
use crate::terminal::print_line;
use colored::Colorize;
use std::collections::HashMap;
use std::fs;
use std::path::PathBuf;
use std::sync::atomic::{AtomicBool, Ordering};
use std::sync::{Arc, Mutex};
use std::time::{Duration, Instant};

mod recording;

/// Truncate a name to fit in a column width, adding ".." if truncated.
fn truncate_name(name: &str, max_len: usize) -> String {
    if name.len() <= max_len {
        name.to_string()
    } else if max_len < 4 {
        name[..max_len].to_string()
    } else {
        format!("{}...", &name[..max_len - 3])
    }
}

// Record/Replay imports
use super::record_replay::{NodeRecorder, NodeReplayer, RecordingConfig, SchedulerRecording};

// Import types from types module
use super::types::RegisteredNode;
use crate::core::NodeMetrics;

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

use super::profiler::RuntimeProfiler;
use super::safety_monitor::SafetyMonitor;

// Auto-optimization capabilities
use super::rt::RuntimeCapabilities;

/// Degradation that occurred during auto-optimization.
///
/// When `Scheduler::new()` auto-applies RT optimizations, it may encounter
/// failures (e.g., no RT permission). These are recorded as degradations
/// rather than errors, allowing the scheduler to still function with
/// reduced capabilities.
#[doc(hidden)]
#[derive(Debug, Clone)]
pub struct RtFeatureDegradation {
    /// What feature was attempted
    pub feature: RtFeature,
    /// What went wrong
    pub reason: String,
    /// Severity of the degradation
    pub severity: DegradationSeverity,
}

/// RT feature that was attempted during auto-optimization.
#[doc(hidden)]
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
#[doc(hidden)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DegradationSeverity {
    /// Significant impact on performance (e.g., no RT priority)
    High,
    /// Moderate impact (e.g., no memory locking)
    Medium,
    /// Minor impact (e.g., no NUMA awareness)
    Low,
}

// =========================================================================
// Sub-structs for Scheduler decomposition
// =========================================================================

/// Runtime capability detection and degradation tracking.
pub(crate) struct RtState {
    pub capabilities: Option<RuntimeCapabilities>,
    pub degradations: Vec<RtFeatureDegradation>,
}

/// Tick-loop timing state.
pub(crate) struct TickState {
    pub period: Duration,
    pub current: u64,
    pub last_instant: Instant,
}

/// Monitoring features: safety, blackbox flight recorder, telemetry, profiling.
///
/// Profiler and blackbox are wrapped in `Arc<Mutex<>>` so they can be shared
/// with executor threads (RT, compute, event, async I/O) for cross-group
/// monitoring. Lock contention is negligible since monitoring calls are fast
/// and executors tick at different cadences.
pub(crate) struct MonitorState {
    pub safety: Option<SafetyMonitor>,
    pub blackbox: Option<Arc<Mutex<super::blackbox::BlackBox>>>,
    pub telemetry: Option<super::telemetry::TelemetryManager>,
    pub profiler: Arc<Mutex<RuntimeProfiler>>,
    pub last_snapshot: Instant,
    pub working_dir: PathBuf,
    /// Pre-allocated buffer for `check_watchdogs()`.
    ///
    /// Reused every tick to avoid the heap allocation that a `-> Vec<String>`
    /// return would require.  Passed by `&mut` into `SafetyMonitor::check_watchdogs`.
    pub watchdog_expired_buf: Vec<String>,
    /// Pre-allocated buffer for graduated watchdog results.
    pub watchdog_graduated_buf: Vec<(String, super::safety_monitor::WatchdogSeverity)>,
}

/// Replay state — only present when replaying a recording.
pub(crate) struct ReplayState {
    pub nodes: HashMap<String, NodeReplayer>,
    pub overrides: HashMap<String, HashMap<String, Vec<u8>>>,
    pub stop_tick: Option<u64>,
    pub speed: f64,
}

/// Recording state — only present when recording a session.
pub(crate) struct RecordingState {
    pub config: RecordingConfig,
    pub scheduler_recording: SchedulerRecording,
}

/// Central orchestrator: holds nodes, drives the tick loop.
pub struct Scheduler {
    pub(super) nodes: Vec<RegisteredNode>,
    pub(super) running: Arc<AtomicBool>,
    pub scheduler_name: String,

    // Grouped sub-structs
    pub(super) tick: TickState,
    pub(super) rt: RtState,
    pub(super) monitor: MonitorState,
    pub(super) replay: Option<ReplayState>,
    pub(super) recording: Option<RecordingState>,

    /// Deferred configuration applied once at `run()` time via `finalize_config()`.
    /// Builder methods mutate this; `finalize_config()` applies it before the tick loop.
    pub(super) pending_config: super::config::SchedulerConfig,

    /// Whether `finalize_and_init()` has been called. Prevents double-init.
    initialized: bool,
}

impl Default for Scheduler {
    fn default() -> Self {
        Self::new()
    }
}

impl Scheduler {
    /// Create a minimal scheduler — **lightweight, no syscalls**.
    ///
    /// Detects runtime capabilities (~30-100μs) but does NOT auto-apply
    /// any OS-level features. Use builder methods to opt in:
    ///
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    ///
    /// // Minimal — just capability detection
    /// let scheduler = Scheduler::new();
    ///
    /// // Configure with builder methods
    /// let scheduler = Scheduler::new().tick_rate(500_u64.hz());
    /// ```
    /// Create a scheduler with default configuration.
    ///
    /// Configuration is deferred until `run()` via builder methods.
    /// Use `.prefer_rt()`, `.require_rt()`, `.watchdog()`, `.blackbox()` to configure.
    pub fn new() -> Self {
        let running = Arc::new(AtomicBool::new(true));
        let now = Instant::now();

        // Detect runtime capabilities (~30-100μs one-time cost)
        let caps = RuntimeCapabilities::detect();

        // Clean up stale SHM namespaces from crashed processes (<1ms)
        let cleanup = crate::memory::platform::cleanup_stale_namespaces();
        if cleanup.removed > 0 {
            log::info!(
                "Cleaned {} stale SHM namespace(s), freed {} bytes",
                cleanup.removed,
                cleanup.bytes_freed
            );
        }

        let config = super::config::SchedulerConfig::default();
        let period = Duration::from_micros((1_000_000.0 / config.timing.global_rate_hz) as u64);

        Self {
            nodes: Vec::new(),
            running,
            scheduler_name: "Scheduler".to_string(),

            tick: TickState {
                period,
                current: 0,
                last_instant: now,
            },
            rt: RtState {
                capabilities: Some(caps),
                degradations: Vec::new(),
            },
            monitor: MonitorState {
                safety: None,
                blackbox: None,
                telemetry: None,
                profiler: Arc::new(Mutex::new(RuntimeProfiler::new_default())),
                last_snapshot: now,
                working_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
                watchdog_expired_buf: Vec::new(),
                watchdog_graduated_buf: Vec::new(),
            },
            replay: None,
            recording: None,
            pending_config: config,
            initialized: false,
        }
    }

    // ========================================================================
    // BUILDER METHODS — composable, chainable, explicit
    // ========================================================================

    /// Try to enable OS-level RT features (mlockall, SCHED_FIFO).
    ///
    /// If the system lacks RT capabilities, logs degradation warnings and
    /// continues running with reduced capabilities. **Never panics.**
    ///
    /// Use `.require_rt()` instead if you need hard failure on non-RT systems.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .prefer_rt()           // try RT, warn if unavailable
    ///     .watchdog(500_u64.ms())
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn prefer_rt(mut self) -> Self {
        self.pending_config.realtime.memory_locking = true;
        self.pending_config.realtime.rt_scheduling_class = true;
        self
    }

    /// Enable OS-level RT features (mlockall, SCHED_FIFO).
    ///
    /// **Panics** if the system lacks RT capabilities. The name `require_rt`
    /// makes this explicit — you chose to fail loudly.
    ///
    /// Use `.prefer_rt()` for graceful degradation instead.
    ///
    /// # Panics
    /// Panics if the system has neither `SCHED_FIFO` nor `mlockall` support.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .require_rt()          // panic if RT unavailable
    ///     .watchdog(500_u64.ms())
    ///     .tick_rate(1000_u64.hz());
    /// ```
    pub fn require_rt(mut self) -> Self {
        let can_rt = self
            .rt
            .capabilities
            .as_ref()
            .is_some_and(|c| c.rt_priority_available || c.mlockall_permitted);
        assert!(
            can_rt,
            "require_rt(): system lacks RT capabilities.\n\
             Use .prefer_rt() to run with graceful degradation instead."
        );
        self.pending_config.realtime.memory_locking = true;
        self.pending_config.realtime.rt_scheduling_class = true;
        self
    }

    /// Enable the watchdog — detects frozen or unresponsive nodes.
    ///
    /// When set, a safety monitor is auto-created that watches all registered
    /// nodes. If a node doesn't tick within the timeout, the watchdog triggers
    /// graduated degradation (warn → reduce rate → isolate → safe state).
    ///
    /// Budget enforcement and deadline monitoring are always active for nodes
    /// that have `.rate()` set (no flag needed).
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .watchdog(500_u64.ms())    // 500ms timeout for frozen nodes
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn watchdog(mut self, timeout: Duration) -> Self {
        self.pending_config.realtime.watchdog_timeout_ms = timeout.as_millis() as u64;
        self
    }

    /// Enable the blackbox flight recorder for crash forensics.
    ///
    /// Records critical events (deadline misses, budget violations, fault tolerance
    /// state changes, emergency stops) in a ring buffer. After a crash, the blackbox
    /// provides a timeline of what happened.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .blackbox(64)              // 64MB flight recorder
    ///     .tick_rate(100_u64.hz());
    /// ```
    pub fn blackbox(mut self, size_mb: usize) -> Self {
        self.pending_config.monitoring.black_box_size_mb = size_mb;
        let bb_dir = self.monitor.working_dir.join(".horus").join("blackbox");
        let bb = super::blackbox::BlackBox::new(size_mb).with_path(bb_dir);
        self.monitor.blackbox = Some(Arc::new(Mutex::new(bb)));
        self
    }

    /// Set the maximum number of deadline misses before emergency stop.
    ///
    /// Default: 100. Only meaningful when nodes have `.rate()` set.
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new()
    ///     .max_deadline_misses(10)   // strict — stop after 10 misses
    ///     .tick_rate(1000_u64.hz());
    /// ```
    pub fn max_deadline_misses(mut self, n: u64) -> Self {
        self.pending_config.realtime.max_deadline_misses = n.max(1);
        self
    }

    /// Set the global tick rate.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus::prelude::*;
    ///
    /// let scheduler = Scheduler::new()
    ///     .tick_rate(1000_u64.hz()); // 1kHz control loop
    /// ```
    pub fn tick_rate(mut self, freq: crate::core::duration_ext::Frequency) -> Self {
        self.pending_config.timing.global_rate_hz = freq.value();
        self
    }

    /// Enable or disable verbose logging from executor threads.
    ///
    /// When disabled, suppresses non-emergency messages from the RT thread
    /// (budget warnings, pre/post-condition failures, startup/shutdown notices).
    /// Emergency-stop messages are always printed regardless of this setting.
    ///
    /// Default: `true` (verbose logging enabled).
    pub fn verbose(mut self, enabled: bool) -> Self {
        self.pending_config.monitoring.verbose = enabled;
        self
    }

    /// Enable recording with sensible defaults.
    ///
    /// Equivalent to applying `SchedulerConfig` with `recording: Some(RecordingConfigYaml::full())`.
    ///
    /// # Example
    /// ```rust,ignore
    /// let mut scheduler = Scheduler::new()
    ///     .tick_rate(100_u64.hz())
    ///     .with_recording();
    /// ```
    pub fn with_recording(mut self) -> Self {
        let recording_yaml = super::config::RecordingConfigYaml::full();
        self.pending_config.recording = Some(recording_yaml.clone());
        let recording_config = super::record_replay::RecordingConfig::from(recording_yaml);
        let scheduler_id = format!(
            "{:x}{:x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            std::process::id() as u64
        );
        let session_name = recording_config.session_name.clone();
        self.recording = Some(RecordingState {
            config: recording_config,
            scheduler_recording: super::record_replay::SchedulerRecording::new(
                &scheduler_id,
                &session_name,
            ),
        });
        self
    }

    // Telemetry: set HORUS_TELEMETRY_ENDPOINT env var.
    // Profiling: always on (negligible overhead).
    // Budget enforcement + deadline monitoring: always on when nodes have .rate() set.

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
    #[doc(hidden)]
    pub fn capabilities(&self) -> Option<&RuntimeCapabilities> {
        self.rt.capabilities.as_ref()
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
    #[doc(hidden)]
    pub fn degradations(&self) -> &[RtFeatureDegradation] {
        &self.rt.degradations
    }

    /// Check if the scheduler has full RT capabilities (no high-severity degradations).
    #[doc(hidden)]
    pub fn has_full_rt(&self) -> bool {
        !self
            .rt
            .degradations
            .iter()
            .any(|d| d.severity == DegradationSeverity::High)
    }

    /// Get a reference to the BlackBox flight recorder.
    ///
    /// The BlackBox automatically records critical events for post-mortem crash analysis:
    /// - Scheduler start/stop
    /// - Node additions
    /// - Deadline misses
    /// - budget violations
    /// - Fault tolerance state changes
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
    #[doc(hidden)]
    pub fn get_blackbox(&self) -> Option<&Arc<Mutex<super::blackbox::BlackBox>>> {
        self.monitor.blackbox.as_ref()
    }

    /// Get safety statistics including budget overruns, deadline misses, and watchdog expirations.
    ///
    /// Returns `None` if the safety monitor is not enabled.
    ///
    /// The returned `SafetyStats` contains:
    /// - `state`: Current safety state (Normal, Degraded, EmergencyStop)
    /// - `budget_overruns`: Number of tick budget violations
    /// - `deadline_misses`: Number of deadline misses
    /// - `watchdog_expirations`: Number of watchdog timeouts
    ///
    /// # Example
    /// ```rust,ignore
    /// let scheduler = Scheduler::new();
    /// // ... run scheduler for a while ...
    ///
    /// if let Some(stats) = scheduler.safety_stats() {
    ///     println!("budget overruns: {}", stats.budget_overruns());
    ///     println!("Deadline misses: {}", stats.deadline_misses());
    /// }
    /// ```
    #[doc(hidden)]
    pub fn safety_stats(&self) -> Option<super::safety_monitor::SafetyStats> {
        self.monitor.safety.as_ref().map(|m| m.get_stats())
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
    #[doc(hidden)]
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
    /// //   [x] Real-time Mode
    /// // ------------------------------------------------------------------
    /// // Safety Features:
    /// //   [ ] Safety Monitor
    /// //   [ ] BlackBox Recorder
    /// // ==================================================================
    /// ```
    pub fn status(&self) -> String {
        let sep = "==================================================================";
        let mut lines = Vec::new();

        // Header
        lines.push(sep.to_string());
        lines.push(format!("SCHEDULER STATUS: {}", self.scheduler_name));
        lines.push(sep.to_string());

        // Sections
        lines.extend(self.platform_status_lines());
        lines.extend(self.safety_status_lines());
        lines.extend(self.node_health_status_lines());

        // Footer
        lines.push(sep.to_string());
        lines.join("\n")
    }

    /// Build platform and RT capability status lines.
    fn platform_status_lines(&self) -> Vec<String> {
        let thin_sep = "------------------------------------------------------------------";
        let mut lines = Vec::new();

        if let Some(caps) = &self.rt.capabilities {
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

        lines
    }

    /// Build safety features, execution mode, and budget statistics status lines.
    fn safety_status_lines(&self) -> Vec<String> {
        let thin_sep = "------------------------------------------------------------------";
        let mut lines = Vec::new();

        // Execution mode
        lines.push(thin_sep.to_string());
        lines.push("Execution Mode:".to_string());
        lines.push("  [x] Real-time Mode".to_string());

        // Safety features
        lines.push(thin_sep.to_string());
        lines.push("Safety Features:".to_string());
        lines.push(format!(
            "  [{}] Safety Monitor",
            if self.monitor.safety.is_some() {
                "x"
            } else {
                " "
            }
        ));
        lines.push(format!(
            "  [{}] BlackBox Recorder",
            if self.monitor.blackbox.is_some() {
                "x"
            } else {
                " "
            }
        ));
        lines.push("  [x] Error Handling".to_string());
        let has_budget =
            self.monitor.safety.is_some() && self.nodes.iter().any(|n| n.tick_budget.is_some());
        lines.push(format!(
            "  [{}] budget Enforcement (RT nodes)",
            if has_budget { "x" } else { " " }
        ));

        // budget Stats
        if let Some(ref monitor) = self.monitor.safety {
            let stats = monitor.get_stats();
            if stats.budget_overruns() > 0 || stats.deadline_misses() > 0 {
                lines.push(thin_sep.to_string());
                lines.push("budget / Deadline Statistics:".to_string());
                if stats.budget_overruns() > 0 {
                    lines.push(format!(
                        "  [WARN] {} budget overruns",
                        stats.budget_overruns()
                    ));
                } else {
                    lines.push("  [OK] No budget overruns".to_string());
                }
                if stats.deadline_misses() > 0 {
                    lines.push(format!(
                        "  [WARN] {} deadline misses",
                        stats.deadline_misses()
                    ));
                } else {
                    lines.push("  [OK] No deadline misses".to_string());
                }
            }
        }

        lines
    }

    /// Build node health and RT degradation status lines.
    fn node_health_status_lines(&self) -> Vec<String> {
        use super::types::NodeHealthState;

        let thin_sep = "------------------------------------------------------------------";
        let mut lines = Vec::new();

        // Node Health
        if !self.nodes.is_empty() {
            lines.push(thin_sep.to_string());
            lines.push("Node Health:".to_string());

            let stopped_count = self.nodes.iter().filter(|n| n.is_stopped).count();
            let warning_count = self
                .nodes
                .iter()
                .filter(|n| n.health_state.load() == NodeHealthState::Warning)
                .count();
            let unhealthy_count = self
                .nodes
                .iter()
                .filter(|n| n.health_state.load() == NodeHealthState::Unhealthy)
                .count();
            let isolated_count = self
                .nodes
                .iter()
                .filter(|n| n.health_state.load() == NodeHealthState::Isolated)
                .count();

            let degraded = stopped_count + warning_count + unhealthy_count + isolated_count;
            if degraded == 0 {
                lines.push(format!("  [OK] All {} nodes healthy", self.nodes.len()));
            } else {
                lines.push(format!(
                    "  {} healthy, {} warning, {} unhealthy, {} isolated, {} stopped",
                    self.nodes.len() - degraded,
                    warning_count,
                    unhealthy_count,
                    isolated_count,
                    stopped_count,
                ));
                for node in &self.nodes {
                    let health = node.health_state.load();
                    if node.is_stopped || health != NodeHealthState::Healthy {
                        let label = if node.is_stopped {
                            "STOPPED".to_string()
                        } else {
                            health.to_string().to_uppercase()
                        };
                        lines.push(format!("    - {}: {}", node.name.as_ref(), label));
                    }
                }
            }
        }

        // Degradations
        if !self.rt.degradations.is_empty() {
            lines.push(thin_sep.to_string());
            lines.push("Degradations:".to_string());
            for deg in &self.rt.degradations {
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

        lines
    }

    /// Apply a full configuration struct. Used internally by horus_py.
    #[doc(hidden)]
    pub fn apply_config(&mut self, config: super::config::SchedulerConfig) {
        // Global tick rate
        let rate_hz =
            if config.timing.global_rate_hz.is_finite() && config.timing.global_rate_hz > 0.0 {
                config.timing.global_rate_hz
            } else {
                100.0 // Safe fallback for invalid rate
            };
        self.tick.period = Duration::from_micros((1_000_000.0 / rate_hz) as u64);

        // RT safety and OS-level optimizations
        self.apply_safety_config(&config.realtime);
        self.apply_rt_optimizations(&config.realtime, &config.resources);

        // Resource configuration
        if let Some(ref cores) = config.resources.cpu_cores {
            print_line(&format!("CPU cores configuration: {:?}", cores));
        }

        // Monitoring (profiling, blackbox, telemetry)
        self.apply_monitoring_config(&config.monitoring);

        // Recording
        if let Some(ref recording_yaml) = config.recording {
            if recording_yaml.enabled {
                self.apply_recording_config(recording_yaml);
            }
        }
    }

    /// Configure the safety monitor for RT nodes (watchdogs, budget, deadlines).
    fn apply_safety_config(&mut self, rt: &super::config::RealTimeConfig) {
        let has_rt_nodes = self.nodes.iter().any(|n| n.is_rt_node);
        let watchdog_active = rt.watchdog_timeout_ms > 0;
        if watchdog_active || has_rt_nodes {
            let monitor = SafetyMonitor::new(rt.max_deadline_misses);

            for registered in self.nodes.iter() {
                if registered.is_rt_node {
                    if watchdog_active {
                        let watchdog_timeout = rt.watchdog_timeout_ms.ms();
                        monitor.add_critical_node(registered.name.to_string(), watchdog_timeout);
                    }

                    if let Some(budget) = registered.tick_budget {
                        monitor.set_tick_budget(registered.name.to_string(), budget);
                    }
                }
            }

            self.monitor.safety = Some(monitor);
            print_line("Safety monitor configured for RT nodes");
        }
    }

    /// Apply OS-level RT optimizations (memory locking, scheduling, affinity, NUMA).
    fn apply_rt_optimizations(
        &mut self,
        rt: &super::config::RealTimeConfig,
        resources: &super::config::ResourceConfig,
    ) {
        use crate::core::rt_config::{RtApplyResult, RtConfig, RtScheduler};

        let has_rt_features =
            rt.memory_locking || rt.rt_scheduling_class || resources.cpu_cores.is_some();

        if has_rt_features {
            let mut builder = RtConfig::builder()
                .memory_locked(rt.memory_locking)
                .warn_on_degradation(true);

            if rt.rt_scheduling_class {
                builder = builder.scheduler(RtScheduler::Fifo).priority(50);
            }

            if let Some(ref cores) = resources.cpu_cores {
                builder = builder.cpu_affinity(cores);
            }

            let rt_config = builder.build();
            match rt_config.apply() {
                Ok(RtApplyResult::FullSuccess) => {
                    if rt.memory_locking {
                        print_line("[SCHEDULER] Memory locked (mlockall)");
                    }
                    if rt.rt_scheduling_class {
                        print_line("[SCHEDULER] RT scheduling enabled (SCHED_FIFO, priority 50)");
                    }
                    if let Some(ref cores) = resources.cpu_cores {
                        print_line(&format!(
                            "[SCHEDULER] CPU affinity set to cores {:?}",
                            cores
                        ));
                    }
                }
                Ok(RtApplyResult::Degraded(degradations)) => {
                    for d in &degradations {
                        print_line(&format!("[SCHEDULER] RT degraded: {:?}", d));
                    }
                }
                Err(e) => {
                    print_line(&format!("[SCHEDULER] RT config error: {}", e));
                }
            }
        }
    }

    /// Apply monitoring configuration (profiling, blackbox, telemetry).
    fn apply_monitoring_config(&mut self, monitoring: &super::config::MonitoringConfig) {
        // Profiling is always on (negligible overhead: ~microsecond per tick via Welford's algorithm)
        self.monitor.profiler.lock().unwrap().enable();

        // Black box flight recorder
        if monitoring.black_box_size_mb > 0 {
            let bb_dir = self.monitor.working_dir.join(".horus").join("blackbox");
            let mut bb =
                super::blackbox::BlackBox::new(monitoring.black_box_size_mb).with_path(bb_dir);
            bb.record(super::blackbox::BlackBoxEvent::SchedulerStart {
                name: self.scheduler_name.clone(),
                node_count: self.nodes.len(),
                config: format!(
                    "rate={}Hz",
                    1_000_000.0 / self.tick.period.as_micros() as f64
                ),
            });
            self.monitor.blackbox = Some(Arc::new(Mutex::new(bb)));
            print_line(&format!(
                "[SCHEDULER] Black box enabled ({}MB buffer)",
                monitoring.black_box_size_mb
            ));
        }

        // Telemetry export
        if let Some(ref endpoint_str) = monitoring.telemetry_endpoint {
            let endpoint = super::telemetry::TelemetryEndpoint::from_string(endpoint_str);
            let interval_ms = 1000u64;
            let mut tm = super::telemetry::TelemetryManager::new(endpoint, interval_ms);
            tm.set_scheduler_name(&self.scheduler_name);
            self.monitor.telemetry = Some(tm);
            print_line(&format!(
                "[SCHEDULER] Telemetry enabled (endpoint: {})",
                endpoint_str
            ));
        }
    }

    /// Apply recording configuration for the record/replay system.
    fn apply_recording_config(&mut self, recording_yaml: &super::config::RecordingConfigYaml) {
        let recording_config = RecordingConfig::from(recording_yaml.clone());
        let session_name = recording_config.session_name.clone();

        // Generate unique scheduler ID
        let scheduler_id = format!(
            "{:x}{:x}",
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_nanos() as u64)
                .unwrap_or(0),
            std::process::id() as u64
        );

        print_line(&format!(
            "[SCHEDULER] Recording enabled (session: {}, compress: {})",
            session_name, recording_yaml.compress
        ));

        self.recording = Some(RecordingState {
            config: recording_config,
            scheduler_recording: SchedulerRecording::new(&scheduler_id, &session_name),
        });
    }

    /// Add a critical node to the safety monitor with a watchdog timeout.
    ///
    /// Critical nodes are monitored more strictly:
    /// - Watchdog expiration triggers emergency stop
    /// - budget violations trigger emergency stop
    /// - Deadline misses trigger emergency stop
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    /// use std::time::Duration;
    ///
    /// let mut scheduler = Scheduler::new().watchdog(500_u64.ms());
    /// // ... after run() starts, safety monitor is active
    /// ```
    ///
    /// # Errors
    /// Returns an error if the safety monitor is not enabled.
    #[doc(hidden)]
    pub fn add_critical_node(
        &mut self,
        node_name: &str,
        watchdog_timeout: std::time::Duration,
    ) -> crate::error::HorusResult<&mut Self> {
        if let Some(ref mut monitor) = self.monitor.safety {
            monitor.add_critical_node(node_name.to_string(), watchdog_timeout);
            Ok(self)
        } else {
            Err(crate::error::HorusError::Config(
                crate::error::ConfigError::Other(
                    "Safety monitor not enabled. Use Scheduler::new().watchdog(500.ms()) to enable."
                        .to_string(),
                ),
            ))
        }
    }

    /// Set the budget (Worst-Case Execution Time) budget for a node.
    ///
    /// When a node exceeds its tick budget:
    /// - Regular nodes: Warning logged, counter incremented
    /// - Critical nodes: Emergency stop triggered
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::Scheduler;
    /// use std::time::Duration;
    ///
    /// let mut scheduler = Scheduler::new().watchdog(500_u64.ms());
    /// // tick budgets are auto-derived from .rate() on the node builder
    /// ```
    ///
    /// # Errors
    /// Returns an error if the safety monitor is not enabled.
    #[doc(hidden)]
    pub fn set_tick_budget(
        &mut self,
        node_name: &str,
        budget: std::time::Duration,
    ) -> crate::error::HorusResult<&mut Self> {
        if let Some(ref mut monitor) = self.monitor.safety {
            monitor.set_tick_budget(node_name.to_string(), budget);
            Ok(self)
        } else {
            Err(crate::error::HorusError::Config(
                crate::error::ConfigError::Other(
                    "Safety monitor not enabled. Use Scheduler::new().watchdog(500.ms()) to enable."
                        .to_string(),
                ),
            ))
        }
    }

    // ============================================================================
    // OS Integration Methods (low-level, genuinely different from config)
    // ============================================================================

    /// Set OS-level scheduling priority using SCHED_FIFO (Linux RT-PREEMPT required).
    ///
    /// Delegates to [`super::rt::set_realtime_priority`].
    ///
    /// # Arguments
    /// * `priority` - Priority level (1-99, higher = more important)
    ///
    /// # Example
    /// ```ignore
    /// scheduler.set_os_priority(99)?;  // Highest priority
    /// ```
    #[doc(hidden)]
    pub fn set_os_priority(&self, priority: i32) -> crate::error::HorusResult<()> {
        if !(1..=99).contains(&priority) {
            return Err(crate::error::HorusError::config(
                "Priority must be between 1 and 99",
            ));
        }
        super::rt::set_realtime_priority(priority)?;
        print_line(&format!(
            "[OK] OS priority set to {} (SCHED_FIFO)",
            priority
        ));
        Ok(())
    }

    /// Pin scheduler to a specific CPU core (prevent context switches).
    ///
    /// Delegates to [`super::rt::set_thread_affinity`].
    ///
    /// # Example
    /// ```ignore
    /// scheduler.pin_to_cpu(7)?;
    /// ```
    #[doc(hidden)]
    pub fn pin_to_cpu(&self, cpu_id: usize) -> crate::error::HorusResult<()> {
        super::rt::set_thread_affinity(&[cpu_id])?;
        print_line(&format!("[OK] Scheduler pinned to CPU core {}", cpu_id));
        Ok(())
    }

    /// Lock all memory pages to prevent page faults (critical for <20μs latency).
    ///
    /// Delegates to [`super::rt::lock_all_memory`].
    ///
    /// # Example
    /// ```ignore
    /// scheduler.lock_memory()?;
    /// ```
    #[doc(hidden)]
    pub fn lock_memory(&self) -> crate::error::HorusResult<()> {
        super::rt::lock_all_memory()?;
        print_line("[OK] Memory locked (no page faults)");
        Ok(())
    }

    /// Pre-fault stack to prevent page faults during execution.
    ///
    /// Delegates to [`super::rt::prefault_stack`].
    ///
    /// # Example
    /// ```ignore
    /// scheduler.prefault_stack(8 * 1024 * 1024)?;  // 8MB stack
    /// ```
    #[doc(hidden)]
    pub fn prefault_stack(&self, stack_size: usize) -> crate::error::HorusResult<()> {
        super::rt::prefault_stack(stack_size)?;
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
    /// Call `.build()` on the builder to register the node.
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
    ///     .build();
    ///
    /// // RT node — auto-detected from .rate()
    /// scheduler.add(MotorController::new())
    ///     .order(0)
    ///     .rate(1000_u64.hz())  // 1kHz → auto-derives budget & deadline
    ///     .build()?;
    ///
    /// // Chain multiple nodes
    /// scheduler.add(SensorNode::new()).order(0).rate(100_u64.hz()).build()?;
    /// scheduler.add(ControlNode::new()).order(1).rate(500_u64.hz()).build()?;
    /// scheduler.add(LoggerNode::new()).order(100).build()?;
    /// ```
    pub fn add<N: Node + 'static>(&mut self, node: N) -> super::node_builder::NodeBuilder<'_> {
        super::node_builder::NodeBuilder::new(self, Box::new(node))
    }

    /// Add a node using a pre-built NodeRegistration.
    ///
    /// This is called internally by `NodeBuilder::done()`. You can also use it
    /// directly with a `NodeRegistration`.
    ///
    /// # Example
    /// ```rust,ignore
    /// use horus_core::scheduling::NodeRegistration;
    ///
    /// let config = NodeRegistration::new(Box::new(my_node))
    ///     .order(0)
    ///     .rate(1000_u64.hz());
    ///
    /// scheduler.add_configured(config);
    /// ```
    #[doc(hidden)]
    pub fn add_configured(&mut self, config: super::node_builder::NodeRegistration) -> &mut Self {
        self.add_configured_internal(config)
    }

    /// Internal method to add a fully configured node.
    fn add_configured_internal(
        &mut self,
        config: super::node_builder::NodeRegistration,
    ) -> &mut Self {
        let node = config.node;
        let priority = config.order;
        let custom_rate = config.rate_hz;
        let is_rt_node = config.is_rt;
        let tick_budget = config.tick_budget;
        let deadline = config.deadline;
        let miss_policy = config.miss_policy;
        let execution_class = config.execution_class;

        let node_name = node.name().to_string();

        let context = NodeInfo::new(node_name.clone());

        // Create node recorder if recording is enabled
        let recorder = if let Some(ref mut rec_state) = self.recording {
            let node_id = format!(
                "{:x}{:x}",
                std::time::SystemTime::now()
                    .duration_since(std::time::UNIX_EPOCH)
                    .map(|d| d.as_nanos() as u64)
                    .unwrap_or(0),
                self.nodes.len() as u64
            );
            let recorder = NodeRecorder::new(&node_name, &node_id, rec_state.config.clone());

            let relative_path = format!("{}@{}.horus", node_name, node_id);
            rec_state
                .scheduler_recording
                .add_node_recording(&node_id, &relative_path);

            Some(recorder)
        } else {
            None
        };

        // Rate comes from builder config only (Node trait no longer declares rate)
        let node_rate = custom_rate;

        // Allocate RtStats for RT nodes
        let rt_stats = if is_rt_node {
            Some(crate::core::RtStats::default())
        } else {
            None
        };

        self.nodes.push(RegisteredNode {
            node,
            name: Arc::from(node_name.as_str()),
            priority,
            initialized: false,
            context: Some(context),
            rate_hz: node_rate,
            last_tick: if node_rate.is_some() {
                Some(Instant::now())
            } else {
                None
            },
            is_rt_node,
            tick_budget,
            deadline,
            recorder,
            is_stopped: false,
            is_paused: false,
            rt_stats,
            miss_policy,
            execution_class,
            health_state: crate::scheduling::types::AtomicHealthState::default(),
        });

        if let Some(rate) = node_rate {
            print_line(&format!(
                "Added {} '{}' with priority {} at {:.1}Hz",
                if is_rt_node { "RT node" } else { "node" },
                node_name,
                priority,
                rate
            ));
            // Ensure tick_period is fast enough for this node's rate
            self.adjust_tick_period_for_node_rates();
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

    /// Tick specific nodes by name (runs continuously with the specified nodes)
    #[doc(hidden)]
    pub fn tick(&mut self, node_names: &[&str]) -> HorusResult<()> {
        // Use the same pattern as run() but with node filtering
        self.run_with_filter(Some(node_names), None)
    }

    /// Check if the scheduler is running
    pub fn is_running(&self) -> bool {
        self.running.load(Ordering::SeqCst)
    }

    /// Stop the scheduler
    pub fn stop(&self) {
        self.running.store(false, Ordering::SeqCst);
    }

    /// Get a clone of the running flag for external stop control.
    ///
    /// This allows external code (e.g., Python bindings) to stop the scheduler
    /// by setting the flag to `false` from outside the run loop.
    #[doc(hidden)]
    pub fn running_flag(&self) -> Arc<AtomicBool> {
        self.running.clone()
    }

    /// Set per-node rate control (chainable)
    ///
    /// Allows individual nodes to run at different frequencies independent of the global scheduler rate.
    /// If a node's rate is not set, it will tick at the global scheduler frequency.
    ///
    /// # Arguments
    /// * `name` - The name of the node
    /// * `rate` - The desired rate as a `Frequency` (e.g. `100_u64.hz()`)
    ///
    /// # Example
    /// ```ignore
    /// scheduler.set_node_rate("sensor", 100_u64.hz());
    /// ```
    #[doc(hidden)]
    pub fn set_node_rate(
        &mut self,
        name: &str,
        rate: crate::core::duration_ext::Frequency,
    ) -> &mut Self {
        let rate_hz = rate.value();
        for registered in self.nodes.iter_mut() {
            if &*registered.name == name {
                registered.rate_hz = Some(rate_hz);
                registered.last_tick = Some(Instant::now());
                print_line(&format!("Set node '{}' rate to {:.1} Hz", name, rate_hz));
                break;
            }
        }
        self
    }

    /// Adjust tick_period to be fast enough for the fastest registered node.
    ///
    /// If a node declares rate_hz(500), the scheduler must tick at >= 500Hz
    /// for that node to actually achieve its declared rate.
    fn adjust_tick_period_for_node_rates(&mut self) {
        let current_rate = 1_000_000.0 / self.tick.period.as_micros() as f64;
        let max_node_rate = self
            .nodes
            .iter()
            .filter_map(|n| n.rate_hz)
            .fold(0.0_f64, f64::max);

        if max_node_rate > current_rate {
            let new_period = max_node_rate.hz().period();
            print_line(&format!(
                "Adjusting scheduler tick rate from {:.1} Hz to {:.1} Hz (fastest node requires it)",
                current_rate, max_node_rate
            ));
            self.tick.period = new_period;
        }
    }

    /// Apply deferred configuration and initialize all nodes.
    ///
    /// Called lazily on first `tick_once()` or at the start of `run()`.
    /// Idempotent — subsequent calls are no-ops.
    fn finalize_and_init(&mut self) {
        if self.initialized {
            return;
        }
        self.finalize_config();
        self.adjust_tick_period_for_node_rates();
        self.initialize_filtered_nodes(None);
        self.initialized = true;
    }

    /// Execute exactly one tick cycle for all nodes, then return.
    ///
    /// Does **not** loop. Does **not** sleep. The caller controls timing.
    /// Lazily initializes on first call (applies deferred config, inits nodes).
    ///
    /// Designed for simulation and testing:
    /// ```rust,ignore
    /// let mut scheduler = Scheduler::new()
    ///     .tick_rate(100_u64.hz());
    ///
    /// scheduler.add(MyNode::new()).build()?;
    ///
    /// // Simulation loop
    /// loop {
    ///     sim.step_physics(dt);
    ///     scheduler.tick_once()?;
    ///     sim.render();
    /// }
    /// ```
    pub fn tick_once(&mut self) -> HorusResult<()> {
        self.finalize_and_init();
        self.execute_single_tick(None)
    }

    /// Execute exactly one tick cycle for specific named nodes only.
    ///
    /// Internal: run one tick cycle. No loop, no sleep.
    fn execute_single_tick(&mut self, node_filter: Option<&[&str]>) -> HorusResult<()> {
        self.process_control_commands();
        self.tick.last_instant = Instant::now();
        self.reinit_pending_nodes();

        // Execute BestEffort nodes synchronously (same logic as execute_nodes but sync)
        self.nodes.sort_by_key(|r| r.priority);
        let num_nodes = self.nodes.len();
        for i in 0..num_nodes {
            if self.execute_single_node(i, node_filter) {
                return Err(crate::horus_internal!(
                    "Fatal node failure during tick_once"
                ));
            }
        }

        if self.check_safety_monitors() {
            return Err(crate::horus_internal!(
                "Emergency stop triggered during tick_once"
            ));
        }

        self.tick.current += 1;
        Ok(())
    }

    /// Main loop with automatic signal handling and cleanup.
    pub fn run(&mut self) -> HorusResult<()> {
        self.run_with_filter(None, None)
    }

    /// Run all nodes for a specified duration, then shutdown gracefully.
    pub fn run_for(&mut self, duration: Duration) -> HorusResult<()> {
        self.run_with_filter(None, Some(duration))
    }

    /// Run specific nodes for a specified duration, then shutdown gracefully.
    #[doc(hidden)]
    pub fn tick_for(&mut self, node_names: &[&str], duration: Duration) -> HorusResult<()> {
        self.run_with_filter(Some(node_names), Some(duration))
    }

    /// Internal method to run scheduler with optional node filtering and duration.
    /// Apply the deferred `pending_config` before the tick loop starts.
    ///
    /// Called once from `run_with_filter()`. This is better than eagerly applying
    /// config in builders because `finalize_config()` runs AFTER all nodes are added,
    /// so `apply_safety_config` correctly sees all nodes.
    fn finalize_config(&mut self) {
        let config = self.pending_config.clone();

        // Apply tick period from deferred config
        if config.timing.global_rate_hz.is_finite() && config.timing.global_rate_hz > 0.0 {
            self.tick.period =
                Duration::from_micros((1_000_000.0 / config.timing.global_rate_hz) as u64);
        }

        // Auto-derive: if any node's rate exceeds the global tick rate, bump it up.
        // This ensures the scheduler is always fast enough for its fastest node,
        // even when tick_hz() wasn't explicitly set high enough.
        self.adjust_tick_period_for_node_rates();

        // Safety config runs here so it sees ALL nodes (added after builder methods)
        self.apply_safety_config(&config.realtime);

        // Auto-enable mlockall if: RT nodes present, system permits it, and user didn't
        // explicitly disable it. This prevents 10-100ms page fault spikes under memory pressure.
        let has_rt_nodes = self
            .nodes
            .iter()
            .any(|n| matches!(n.execution_class, super::types::ExecutionClass::Rt));
        let mut rt_config = config.realtime.clone();
        if has_rt_nodes && !rt_config.memory_locking {
            if let Some(ref caps) = self.rt.capabilities {
                if caps.mlockall_permitted {
                    rt_config.memory_locking = true;
                    self.pending_config.realtime.memory_locking = true;
                }
            }
        }

        // Env var: HORUS_RT_CORES — comma-separated core IDs (e.g. "2,3")
        // Only applies if no cpu_cores already configured.
        if self.pending_config.resources.cpu_cores.is_none() {
            if let Ok(cores_str) = std::env::var("HORUS_RT_CORES") {
                let cores: Vec<usize> = cores_str
                    .split(',')
                    .filter_map(|s| s.trim().parse::<usize>().ok())
                    .collect();
                if !cores.is_empty() {
                    self.pending_config.resources.cpu_cores = Some(cores);
                }
            }
        }

        // Re-read resources after env var override
        let resources = self.pending_config.resources.clone();
        self.apply_rt_optimizations(&rt_config, &resources);

        // Env var: HORUS_TELEMETRY_ENDPOINT — UDP or file URI (e.g. "udp://localhost:9999")
        if self.pending_config.monitoring.telemetry_endpoint.is_none() {
            if let Ok(endpoint) = std::env::var("HORUS_TELEMETRY_ENDPOINT") {
                if !endpoint.is_empty() {
                    self.pending_config.monitoring.telemetry_endpoint = Some(endpoint);
                }
            }
        }

        // Re-read monitoring config after env var overrides
        let monitoring_config = self.pending_config.monitoring.clone();

        // Only apply monitoring if not already set by eager builders (with_blackbox, etc.)
        if self.monitor.blackbox.is_none() {
            self.apply_monitoring_config(&monitoring_config);
        } else {
            // Blackbox already exists (from .blackbox() builder), only apply profiling/telemetry
            if monitoring_config.verbose {
                self.monitor.profiler.lock().unwrap().enable();
            }
            if let Some(ref endpoint_str) = monitoring_config.telemetry_endpoint {
                if self.monitor.telemetry.is_none() {
                    let endpoint = super::telemetry::TelemetryEndpoint::from_string(endpoint_str);
                    let mut tm = super::telemetry::TelemetryManager::new(endpoint, 1000u64);
                    tm.set_scheduler_name(&self.scheduler_name);
                    self.monitor.telemetry = Some(tm);
                }
            }
        }

        if let Some(ref recording_yaml) = config.recording {
            if recording_yaml.enabled && self.recording.is_none() {
                self.apply_recording_config(recording_yaml);
            }
        }
    }

    fn run_with_filter(
        &mut self,
        node_filter: Option<&[&str]>,
        duration: Option<Duration>,
    ) -> HorusResult<()> {
        // Use a single-threaded tokio runtime for the scheduler main loop.
        // The main loop only needs timers (tokio::time::sleep). Heavy async I/O
        // nodes run on their own dedicated runtime (see AsyncExecutor).
        let rt = tokio::runtime::Builder::new_current_thread()
            .enable_time()
            .build()
            .horus_context("creating scheduler tokio runtime")?;

        rt.block_on(async {
            let start_time = Instant::now();

            self.finalize_config();
            self.initialized = true;
            self.install_panic_hook();
            self.setup_signal_handlers();
            self.initialize_filtered_nodes(node_filter);
            Self::setup_control_directory();
            self.update_registry();
            self.adjust_tick_period_for_node_rates();

            let deterministic = self.pending_config.timing.deterministic_order;

            // In deterministic mode: all nodes stay on the main thread,
            // no executor threads are spawned. This guarantees identical
            // execution order across runs.
            let mut rt_executor = None;
            let mut compute_executor = None;
            let mut event_executor = None;
            let mut async_executor = None;

            if deterministic {
                print_line("Deterministic mode: all nodes execute sequentially on main thread");
                // All nodes stay in self.nodes — no grouping, no executors
            } else {
                // Group nodes by ExecutionClass and dispatch to dedicated executors:
                // - RT nodes → RtExecutor (dedicated high-priority thread)
                // - Compute nodes → ComputeExecutor (parallel thread pool)
                // - Event nodes → EventExecutor (per-node watcher threads)
                // - AsyncIo nodes → AsyncExecutor (tokio blocking pool)
                // - BestEffort nodes → stay in self.nodes for main-thread sequential execution
                let all_nodes = std::mem::take(&mut self.nodes);
                let groups = super::types::group_nodes_by_class(all_nodes);

                // BestEffort nodes remain on the main thread
                self.nodes = groups.main_nodes;

                // Shared monitors for all executor threads
                let shared_monitors = super::types::SharedMonitors {
                    profiler: self.monitor.profiler.clone(),
                    blackbox: self.monitor.blackbox.clone(),
                    verbose: self.pending_config.monitoring.verbose,
                };

                if !groups.rt_nodes.is_empty() {
                    print_line(&format!(
                        "Starting RT executor with {} RT nodes on dedicated thread",
                        groups.rt_nodes.len()
                    ));
                    let rt_cpus = if let Some(ref cores) = self.pending_config.resources.cpu_cores {
                        cores.clone()
                    } else if let Some(ref caps) = self.rt.capabilities {
                        if !caps.recommended_rt_cpus.is_empty() {
                            caps.recommended_rt_cpus.clone()
                        } else if caps.cpu_count > 1 {
                            vec![caps.cpu_count - 1]
                        } else {
                            Vec::new()
                        }
                    } else {
                        Vec::new()
                    };

                    rt_executor = Some(super::rt_executor::RtExecutor::start(
                        groups.rt_nodes,
                        self.running.clone(),
                        self.tick.period,
                        shared_monitors.clone(),
                        rt_cpus,
                    ));
                }

                if !groups.compute_nodes.is_empty() {
                    print_line(&format!(
                        "Starting compute executor with {} nodes on thread pool",
                        groups.compute_nodes.len()
                    ));
                    compute_executor = Some(super::compute_executor::ComputeExecutor::start(
                        groups.compute_nodes,
                        self.running.clone(),
                        self.tick.period,
                        shared_monitors.clone(),
                    ));
                }

                if !groups.event_nodes.is_empty() {
                    print_line(&format!(
                        "Starting event executor with {} event-triggered nodes",
                        groups.event_nodes.len()
                    ));
                    event_executor = Some(super::event_executor::EventExecutor::start(
                        groups.event_nodes,
                        self.running.clone(),
                        shared_monitors.clone(),
                    ));
                }

                if !groups.async_io_nodes.is_empty() {
                    print_line(&format!(
                        "Starting async I/O executor with {} nodes on tokio pool",
                        groups.async_io_nodes.len()
                    ));
                    async_executor = Some(super::async_executor::AsyncExecutor::start(
                        groups.async_io_nodes,
                        self.running.clone(),
                        self.tick.period,
                        shared_monitors.clone(),
                    ));
                }
            }

            // Main tick loop
            while self.is_running() {
                if self.should_stop_loop(start_time, duration) {
                    break;
                }
                self.process_control_commands();
                self.tick.last_instant = Instant::now();
                self.reinit_pending_nodes();
                self.execute_nodes(node_filter).await;
                if self.check_safety_monitors() {
                    break;
                }
                self.periodic_monitoring(start_time);
                if let Some(sleep_dur) = self.compute_tick_sleep() {
                    tokio::time::sleep(sleep_dur).await;
                }
                self.advance_tick();
            }

            // Stop executors and reclaim nodes for shutdown
            self.running.store(false, Ordering::SeqCst);
            if let Some(executor) = rt_executor {
                let rt_nodes = executor.stop();
                self.nodes.extend(rt_nodes);
            }
            if let Some(executor) = compute_executor {
                let compute_nodes = executor.stop();
                self.nodes.extend(compute_nodes);
            }
            if let Some(executor) = event_executor {
                let event_nodes = executor.stop();
                self.nodes.extend(event_nodes);
            }
            if let Some(executor) = async_executor {
                let async_nodes = executor.stop();
                self.nodes.extend(async_nodes);
            }

            self.shutdown_filtered_nodes(node_filter);
            self.finalize_run();
        });

        Ok(())
    }

    /// Set up Ctrl+C and SIGTERM signal handlers for graceful shutdown.
    /// Install a global panic hook for forensic logging.
    ///
    /// Chains with the previous hook and adds:
    /// - Blackbox flush to disk (if configured)
    /// - Crash report written to `<temp_dir>/horus_crash_<pid>.log`
    fn install_panic_hook(&self) {
        let blackbox = self.monitor.blackbox.clone();
        let scheduler_name = self.scheduler_name.clone();
        let prev_hook = std::panic::take_hook();

        std::panic::set_hook(Box::new(move |info| {
            // 1. Flush blackbox to disk
            if let Some(ref bb) = blackbox {
                if let Ok(mut bb) = bb.try_lock() {
                    bb.flush_wal();
                }
            }

            // 2. Build crash report
            let thread = std::thread::current();
            let thread_name = thread.name().unwrap_or("<unnamed>");
            let pid = std::process::id();
            let location = if let Some(loc) = info.location() {
                format!("{}:{}:{}", loc.file(), loc.line(), loc.column())
            } else {
                "unknown location".to_string()
            };
            let payload = if let Some(s) = info.payload().downcast_ref::<&str>() {
                s.to_string()
            } else if let Some(s) = info.payload().downcast_ref::<String>() {
                s.clone()
            } else {
                "unknown panic payload".to_string()
            };

            let report = format!(
                "=== HORUS Crash Report ===\n\
                 Scheduler: {}\n\
                 PID: {}\n\
                 Thread: {}\n\
                 Location: {}\n\
                 Panic: {}\n\
                 Blackbox flushed: {}\n\
                 ===========================\n",
                scheduler_name,
                pid,
                thread_name,
                location,
                payload,
                blackbox.is_some(),
            );

            // 3. Write crash report to /tmp (best-effort, no allocation beyond the format above)
            let crash_path = std::env::temp_dir().join(format!("horus_crash_{}.log", pid));
            let _ = std::fs::write(&crash_path, &report);

            // 4. Also print to stderr (visible in logs)
            eprintln!("{}", report);

            // 5. Chain to previous hook
            prev_hook(info);
        }));
    }

    fn setup_signal_handlers(&self) {
        let running = self.running.clone();
        if let Err(e) = ctrlc::set_handler(move || {
            print_line("\nCtrl+C received! Shutting down HORUS scheduler...");
            running.store(false, Ordering::SeqCst);
            std::thread::spawn(|| {
                std::thread::sleep(2_u64.secs());
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
        // SAFETY: SIGTERM is a valid signal; sigterm_handler is a valid function pointer.
        unsafe {
            libc::signal(
                libc::SIGTERM,
                sigterm_handler as *const () as libc::sighandler_t,
            );
        }
    }

    /// Initialize nodes matching the optional filter.
    fn initialize_filtered_nodes(&mut self, node_filter: Option<&[&str]>) {
        for registered in self.nodes.iter_mut() {
            let node_name = registered.name.as_ref();
            let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

            if should_run && !registered.initialized {
                if let Some(ref mut ctx) = registered.context {
                    // Set node context for hlog!() macro
                    set_node_context(node_name, 0);
                    let init_result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.init()
                        }));
                    clear_node_context();

                    // Convert panic to error
                    let init_result = match init_result {
                        Ok(result) => result,
                        Err(_) => Err(crate::HorusError::Node(
                            crate::error::NodeError::InitPanic {
                                node: node_name.to_string(),
                            },
                        )),
                    };

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
                            if let Some(hint) = e.help() {
                                print_line(&format!(
                                    "Failed to initialize node '{}': {}\n  hint: {}",
                                    node_name, e, hint
                                ));
                            } else {
                                print_line(&format!(
                                    "Failed to initialize node '{}': {}",
                                    node_name, e
                                ));
                            }

                            let severity = e.severity();
                            if severity == crate::error::Severity::Fatal {
                                print_line(&format!(
                                    " FATAL: Node '{}' init failed with fatal severity — stopping",
                                    node_name
                                ));
                                ctx.transition_to_crashed(format!("Fatal init: {}", e));
                                self.stop();
                                return;
                            } else if severity == crate::error::Severity::Transient {
                                ctx.transition_to_error(format!(
                                    "Init failed (transient, will retry): {}",
                                    e
                                ));
                            } else {
                                ctx.transition_to_error(format!("Initialization failed: {}", e));
                            }
                        }
                    }
                }
            }
        }
    }

    /// Check if the main loop should stop (duration limit, replay stop tick, or SIGTERM).
    fn should_stop_loop(&self, start_time: Instant, duration: Option<Duration>) -> bool {
        if let Some(max_duration) = duration {
            if start_time.elapsed() >= max_duration {
                print_line(&format!(
                    "Scheduler reached time limit of {:?}",
                    max_duration
                ));
                return true;
            }
        }

        if let Some(stop_tick) = self.replay.as_ref().and_then(|r| r.stop_tick) {
            if self.tick.current >= stop_tick {
                print_line(&format!("[REPLAY] Reached stop tick {}", stop_tick));
                return true;
            }
        }

        if SIGTERM_RECEIVED.load(Ordering::SeqCst) {
            print_line("\nSIGTERM received! Shutting down HORUS scheduler...");
            return true;
        }

        false
    }

    /// Re-initialize nodes that need restart (set by control commands).
    fn reinit_pending_nodes(&mut self) {
        for registered in self.nodes.iter_mut() {
            if !registered.is_stopped && !registered.is_paused && !registered.initialized {
                let node_name = registered.name.as_ref();
                if let Some(ref mut ctx) = registered.context {
                    // Set node context for hlog!() macro
                    set_node_context(node_name, 0);
                    let init_result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.init()
                        }));
                    clear_node_context();

                    // Convert panic to error
                    let init_result = match init_result {
                        Ok(result) => result,
                        Err(_) => Err(crate::HorusError::Node(
                            crate::error::NodeError::ReInitPanic {
                                node: node_name.to_string(),
                            },
                        )),
                    };

                    match init_result {
                        Ok(()) => {
                            registered.initialized = true;
                            print_line(&format!("[CONTROL] Node '{}' re-initialized", node_name));
                        }
                        Err(e) => {
                            if let Some(hint) = e.help() {
                                print_line(&format!(
                                    "[CONTROL] Failed to re-initialize node '{}': {}\n  hint: {}",
                                    node_name, e, hint
                                ));
                            } else {
                                print_line(&format!(
                                    "[CONTROL] Failed to re-initialize node '{}': {}",
                                    node_name, e
                                ));
                            }

                            let severity = e.severity();
                            if severity == crate::error::Severity::Fatal {
                                registered.is_stopped = true;
                                ctx.transition_to_crashed(format!("Fatal re-init: {}", e));
                            } else if severity == crate::error::Severity::Transient {
                                ctx.transition_to_error(format!(
                                    "Re-init failed (transient, will retry): {}",
                                    e
                                ));
                            } else {
                                registered.is_stopped = true;
                                ctx.transition_to_error(format!("Re-initialization failed: {}", e));
                            }
                        }
                    }
                }
            }
        }
    }

    /// Check watchdogs with graduated severity and update node health states.
    ///
    /// Uses graduated watchdog to transition nodes through health states:
    /// - Warning (1x timeout): log, node stays Healthy→Warning
    /// - Expired (2x timeout): mark Unhealthy, skip in tick loop
    /// - Critical (3x timeout): mark Isolated, call `enter_safe_state()` for critical nodes
    ///
    /// Returns `true` if the scheduler should stop (emergency stop triggered).
    fn check_safety_monitors(&mut self) -> bool {
        if let Some(ref monitor) = self.monitor.safety {
            // Use graduated check for per-node health state transitions
            monitor
                .check_watchdogs_graduated(&mut self.monitor.watchdog_graduated_buf);

            // Apply health state transitions based on severity
            for (node_name, severity) in &self.monitor.watchdog_graduated_buf {
                use super::safety_monitor::WatchdogSeverity;
                use super::types::NodeHealthState;

                if let Some(registered) = self.nodes.iter_mut().find(|n| n.name.as_ref() == node_name) {
                    let current = registered.health_state.load();
                    match severity {
                        WatchdogSeverity::Ok => {
                            // Should not appear in results, but handle gracefully
                        }
                        WatchdogSeverity::Warning => {
                            if current == NodeHealthState::Healthy {
                                registered.health_state.store(NodeHealthState::Warning);
                                print_line(&format!(
                                    " Watchdog warning: '{}' (1x timeout) — marking Warning",
                                    node_name
                                ));
                            }
                        }
                        WatchdogSeverity::Expired => {
                            if current != NodeHealthState::Unhealthy
                                && current != NodeHealthState::Isolated
                            {
                                registered.health_state.store(NodeHealthState::Unhealthy);
                                print_line(&format!(
                                    " Watchdog expired: '{}' (2x timeout) — marking Unhealthy, skipping",
                                    node_name
                                ));
                            }
                        }
                        WatchdogSeverity::Critical => {
                            if current != NodeHealthState::Isolated {
                                registered.health_state.store(NodeHealthState::Isolated);
                                registered.node.enter_safe_state();
                                print_line(&format!(
                                    " Watchdog critical: '{}' (3x timeout) — Isolated, entered safe state",
                                    node_name
                                ));
                            }
                        }
                    }
                }
            }

            // Also run the classic expired check for backward compat logging
            monitor.check_watchdogs(&mut self.monitor.watchdog_expired_buf);

            if monitor.is_emergency_stop() {
                print_line(" Emergency stop activated - shutting down scheduler");
                if let Some(ref bb) = self.monitor.blackbox {
                    bb.lock()
                        .unwrap()
                        .record(super::blackbox::BlackBoxEvent::EmergencyStop {
                            reason: "Safety monitor triggered emergency stop".to_string(),
                        });
                }

                return true;
            }
        }
        false
    }

    /// Periodic registry snapshot, failure logging, blackbox tick, and telemetry export.
    fn periodic_monitoring(&mut self, start_time: Instant) {
        // Registry snapshot every 5 seconds
        if self.monitor.last_snapshot.elapsed() >= 5_u64.secs() {
            self.snapshot_state_to_registry();
            self.monitor.last_snapshot = Instant::now();

            // Log stopped nodes
            let stopped_count = self.nodes.iter().filter(|n| n.is_stopped).count();
            if stopped_count > 0 {
                if let Some(ref bb) = self.monitor.blackbox {
                    bb.lock()
                        .unwrap()
                        .record(super::blackbox::BlackBoxEvent::Custom {
                            category: "safety".to_string(),
                            message: format!("{} nodes stopped", stopped_count),
                        });
                }
            }
        }

        // === Runtime feature integrations ===

        // Black box tick increment
        if let Some(ref bb) = self.monitor.blackbox {
            bb.lock().unwrap().tick();
        }

        // Telemetry export (if interval elapsed)
        if let Some(ref mut tm) = self.monitor.telemetry {
            if tm.should_export() {
                let profiler = self.monitor.profiler.lock().unwrap();
                let total_ticks = profiler
                    .node_stats
                    .values()
                    .map(|s| s.count)
                    .max()
                    .unwrap_or(0) as u64;
                tm.counter("scheduler_ticks", total_ticks);
                tm.gauge("scheduler_uptime_secs", start_time.elapsed().as_secs_f64());
                tm.gauge("nodes_active", self.nodes.len() as f64);

                for registered in &self.nodes {
                    let node_name = registered.name.as_ref();
                    if let Some(stats) = profiler.node_stats.get(node_name) {
                        let mut labels = std::collections::HashMap::new();
                        labels.insert("node".to_string(), node_name.to_string());
                        tm.gauge_with_labels("node_avg_duration_us", stats.avg_us, labels.clone());
                        tm.counter_with_labels("node_tick_count", stats.count as u64, labels);
                    }
                }
                drop(profiler);

                let _ = tm.export();
            }
        }
    }

    /// Compute the tick sleep duration.
    fn compute_tick_sleep(&self) -> Option<Duration> {
        let sleep_duration = if let Some(ref replay) = self.replay {
            if replay.speed != 1.0 {
                Duration::from_nanos((self.tick.period.as_nanos() as f64 / replay.speed) as u64)
            } else {
                self.tick.period
            }
        } else {
            self.tick.period
        };

        Some(sleep_duration)
    }

    /// Increment tick counter.
    fn advance_tick(&mut self) {
        self.tick.current += 1;
    }

    /// Shutdown nodes matching the optional filter.
    fn shutdown_filtered_nodes(&mut self, node_filter: Option<&[&str]>) {
        for registered in self.nodes.iter_mut() {
            let node_name = registered.name.as_ref();
            let should_run = node_filter.is_none_or(|filter| filter.contains(&node_name));

            if should_run && registered.initialized {
                if let Some(ref mut ctx) = registered.context {
                    ctx.record_shutdown();

                    // Set node context for hlog!() macro
                    set_node_context(node_name, ctx.metrics().total_ticks());
                    let shutdown_result =
                        std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                            registered.node.shutdown()
                        }));
                    clear_node_context();

                    // Convert panic to error
                    let shutdown_result = match shutdown_result {
                        Ok(result) => result,
                        Err(_) => Err(crate::HorusError::Node(
                            crate::error::NodeError::ShutdownPanic {
                                node: node_name.to_string(),
                            },
                        )),
                    };

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
                            print_line(&format!("Error shutting down node '{}': {}", node_name, e));
                        }
                    }
                }
            }
        }
    }

    /// Record final blackbox/telemetry events and clean up registry/session files.
    fn finalize_run(&mut self) {
        let total_ticks = self
            .monitor
            .profiler
            .lock()
            .unwrap()
            .node_stats
            .values()
            .map(|s| s.count)
            .max()
            .unwrap_or(0) as u64;

        // Record scheduler stop to blackbox and save
        if let Some(ref bb) = self.monitor.blackbox {
            let mut bb = bb.lock().unwrap();
            bb.record(super::blackbox::BlackBoxEvent::SchedulerStop {
                reason: "Normal shutdown".to_string(),
                total_ticks,
            });
            // Flush any buffered WAL records that haven't reached the batch
            // interval yet — prevents losing the final events on clean exit.
            bb.flush_wal();
            if let Err(e) = bb.save() {
                print_line(&format!("[BLACKBOX] Failed to save: {}", e));
            }
        }

        // Final telemetry export
        if let Some(ref mut tm) = self.monitor.telemetry {
            tm.counter("scheduler_ticks", total_ticks);
            tm.gauge("scheduler_shutdown", 1.0);
            let _ = tm.export();
        }

        // Print timing report if profiling enabled and ticks were executed
        if self.pending_config.monitoring.verbose && total_ticks > 0 {
            self.print_timing_report();
        }

        // Clean up registry file and session
        self.cleanup_registry();
        Self::cleanup_session();

        print_line("Scheduler shutdown complete");
    }

    /// Print a formatted per-node timing report to stderr.
    ///
    /// Uses data from: RuntimeProfiler (Welford avg/stddev/min/max) and
    /// SafetyMonitor (ring buffer p99, budget, overruns, deadline misses).
    ///
    /// Zero runtime cost — only runs on shutdown. Output goes to stderr
    /// to avoid polluting stdout piping.
    fn print_timing_report(&self) {
        let profiler = self.monitor.profiler.lock().unwrap();
        if profiler.node_stats.is_empty() {
            return;
        }

        // Gather safety monitor data (if available)
        let safety_data: HashMap<String, (super::safety_monitor::TimingStats, Option<Duration>, u64)> =
            if let Some(ref monitor) = self.monitor.safety {
                monitor
                    .all_node_timing()
                    .into_iter()
                    .map(|(name, stats, budget, overruns)| (name, (stats, budget, overruns)))
                    .collect()
            } else {
                HashMap::new()
            };

        let sep = "=".repeat(90);
        let thin_sep = "-".repeat(90);
        eprintln!("\n{}", sep);
        eprintln!("TIMING REPORT (per-node)");
        eprintln!("{}", sep);
        eprintln!(
            "{:<20} {:>8} {:>8} {:>8} {:>8} {:>10} {:>8} {:>8}",
            "Node", "Avg(us)", "P99(us)", "Max(us)", "Stddev", "Budget(us)", "Overruns", "Misses"
        );
        eprintln!("{}", thin_sep);

        // Sort nodes by name for consistent output
        let mut node_names: Vec<&String> = profiler.node_stats.keys().collect();
        node_names.sort();

        let mut suggestions: Vec<String> = Vec::new();

        for name in &node_names {
            let stats = &profiler.node_stats[*name];
            if stats.count == 0 {
                continue;
            }

            let (p99_str, budget_str, overruns_str, misses_str) =
                if let Some((ref ring_stats, budget, overruns)) = safety_data.get(*name) {
                    let p99 = format!("{}", ring_stats.p99_us);
                    let budget_s = budget
                        .map(|b| format!("{}", b.as_micros()))
                        .unwrap_or_else(|| "-".to_string());
                    let overruns_s = format!("{}", overruns);
                    let misses_s = format!("{}", ring_stats.total_ticks); // reuse total_ticks
                    (p99, budget_s, overruns_s, misses_s)
                } else {
                    ("-".to_string(), "-".to_string(), "0".to_string(), "-".to_string())
                };

            // Status indicator
            let status = if let Some((_, _, overruns)) = safety_data.get(*name) {
                if *overruns > 0 { "!" } else { "" }
            } else {
                ""
            };

            eprintln!(
                "{:<20} {:>8.0} {:>8} {:>8.0} {:>8.1} {:>10} {:>8} {:>7}{}",
                truncate_name(name, 20),
                stats.avg_us,
                p99_str,
                stats.max_us,
                stats.stddev_us,
                budget_str,
                overruns_str,
                misses_str,
                status,
            );

            // Generate suggestions
            if let Some((ref ring_stats, budget, overruns)) = safety_data.get(*name) {
                if budget.is_none() && ring_stats.p99_us > 0 {
                    suggestions.push(format!(
                        "  {} — no budget set (min={}us avg={}us p99={}us max={}us). Use .rate() to auto-derive budget",
                        name, ring_stats.min_us, ring_stats.avg_us, ring_stats.p99_us, ring_stats.max_us
                    ));
                }
                if *overruns > 10 {
                    suggestions.push(format!(
                        "  {} — {} overruns. Consider increasing budget or reducing tick complexity",
                        name, overruns
                    ));
                }
            }
        }

        eprintln!("{}", thin_sep);
        eprintln!(
            "Total nodes: {}  |  Ticked: {}",
            self.nodes.len(),
            node_names.len()
        );

        if !suggestions.is_empty() {
            eprintln!("\nSuggestions:");
            for s in &suggestions {
                eprintln!("{}", s);
            }
        }

        eprintln!("{}\n", sep);
    }

    /// Get information about all registered nodes
    #[doc(hidden)]
    pub fn node_list(&self) -> Vec<String> {
        self.nodes
            .iter()
            .map(|registered| registered.name.to_string())
            .collect()
    }

    /// Get performance metrics for all nodes.
    pub fn metrics(&self) -> Vec<NodeMetrics> {
        self.nodes
            .iter()
            .map(|registered| {
                let name = registered.name.to_string();
                let order = registered.priority;

                if let Some(ref ctx) = registered.context {
                    ctx.metrics().snapshot(name, order)
                } else {
                    NodeMetrics::new(name, order)
                }
            })
            .collect()
    }

    /// Get real-time statistics for a specific node.
    ///
    /// Returns `None` if the node doesn't exist or is not an RT node.
    #[doc(hidden)]
    pub fn rt_stats(&self, node_name: &str) -> Option<&crate::core::RtStats> {
        self.nodes
            .iter()
            .find(|n| &*n.name == node_name)
            .and_then(|n| n.rt_stats.as_ref())
    }

    /// Write metadata to registry file for monitor to read
    fn update_registry(&self) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();

            // Collect pub/sub info from each node
            let nodes_json: Vec<String> = self.nodes.iter().map(|registered| {
                let name = registered.name.as_ref();
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
                self.monitor.working_dir.to_string_lossy(),
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
                                if registered.name.as_ref() == node_name {
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
        // Clean up this process's SHM namespace on exit.
        // On normal exit, the namespace persists until the process group dies.
        // On signal-triggered exit, we proactively clean stale namespaces
        // so a restart doesn't see orphaned SHM files.
        let cleanup = crate::memory::platform::cleanup_stale_namespaces();
        if cleanup.removed > 0 {
            log::info!(
                "Session cleanup: removed {} stale SHM namespace(s), freed {} bytes",
                cleanup.removed,
                cleanup.bytes_freed
            );
        }
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
                let name = registered.name.as_ref();
                let priority = registered.priority;

                // Get pub/sub from Node trait (macro-declared)
                // Runtime-discovered pub/sub is now tracked by TopicRegistry, not NodeInfo
                let publishers = registered.node.publishers();
                let subscribers = registered.node.subscribers();

                // Get state and health from context
                let (state_str, health_str, error_count, tick_count) = if let Some(ref ctx) = registered.context {
                    let metrics = ctx.metrics();
                    // Watchdog health overrides context-based health when degraded
                    let watchdog_health = registered.health_state.load();
                    let health = if watchdog_health != super::types::NodeHealthState::Healthy {
                        watchdog_health.to_string()
                    } else {
                        metrics.calculate_health().as_str().to_string()
                    };
                    (
                        ctx.state().to_string(),
                        health,
                        metrics.errors_count(),
                        metrics.total_ticks(),
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
                self.monitor.working_dir.to_string_lossy(),
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

    /// Execute a single node: tick + profiling + budget + deadline + failure handling.
    /// Returns true if the scheduler should stop (fatal failure).
    fn execute_single_node(&mut self, i: usize, node_filter: Option<&[&str]>) -> bool {
        // Skip stopped nodes
        if self.nodes[i].is_stopped {
            return false;
        }

        // Skip Unhealthy/Isolated nodes (watchdog has marked them for skipping)
        {
            use super::types::NodeHealthState;
            let health = self.nodes[i].health_state.load();
            if matches!(health, NodeHealthState::Unhealthy | NodeHealthState::Isolated) {
                return false;
            }
        }

        // Auto-unpause nodes that were paused by Miss::Skip
        // (they skip exactly one tick, then resume)
        if self.nodes[i].is_paused {
            self.nodes[i].is_paused = false;
            return false;
        }

        let (should_run, should_tick) = {
            let registered = &self.nodes[i];
            let name = registered.name.as_ref();
            let should_run = node_filter.is_none_or(|filter| filter.contains(&name));

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

            (should_run, should_tick)
        };

        if !should_tick {
            return false;
        }

        // Update last tick time if rate limited
        if self.nodes[i].rate_hz.is_some() {
            self.nodes[i].last_tick = Some(Instant::now());
        }

        if should_run && self.nodes[i].initialized {
            // Feed watchdog for RT nodes
            if self.nodes[i].is_rt_node {
                if let Some(ref monitor) = self.monitor.safety {
                    monitor.feed_watchdog(&self.nodes[i].name);
                }
            }

            // Begin recording tick (before node execution)
            {
                if let Some(ref mut recorder) = self.nodes[i].recorder {
                    recorder.begin_tick(self.tick.current);
                }
            }

            // Capture inputs from subscriber topics (for recording)
            {
                let RegisteredNode { ref mut node, ref mut recorder, .. } = self.nodes[i];
                if let Some(recorder) = recorder.as_mut() {
                    if recorder.is_active_tick() {
                        let subscribers = node.subscribers();
                        if !subscribers.is_empty() {
                            let topics_dir = crate::memory::platform::shm_topics_dir();
                            for sub in &subscribers {
                                let topic_path = topics_dir.join(&sub.topic_name);
                                if let Some(slot_read) = crate::communication::read_latest_slot_bytes(&topic_path, 0) {
                                    recorder.record_input(&sub.topic_name, slot_read.payload);
                                }
                            }
                        }
                    }
                }
            }

            // Replay: if this node has a replayer, advance it and feed the
            // recorded (or overridden) outputs into the node's recorder so
            // that diff/export tools can compare replay results.
            //
            // NOTE: This does NOT yet publish replay data to shared-memory
            // topics for live subscriber nodes.  Full shm injection requires
            // a `write_topic_slot_bytes` counterpart and will be added in a
            // follow-up task.
            {
                let node_name = self.nodes[i].name.clone();
                let replay_outputs: Option<Vec<(String, Vec<u8>)>> = self.replay.as_mut().and_then(|replay| {
                    let replayer = replay.nodes.get_mut(node_name.as_ref())?;
                    let node_overrides = replay.overrides.get(node_name.as_ref());
                    let snapshot = replayer.current_snapshot()?;
                    let outputs: Vec<(String, Vec<u8>)> = snapshot.outputs.iter().map(|(topic, data)| {
                        let output_data = node_overrides
                            .and_then(|ovr| ovr.get(topic))
                            .unwrap_or(data)
                            .clone();
                        (topic.clone(), output_data)
                    }).collect();
                    replayer.advance();
                    Some(outputs)
                });
                if let (Some(outputs), Some(ref mut recorder)) =
                    (replay_outputs, self.nodes[i].recorder.as_mut())
                {
                    for (topic, data) in outputs {
                        recorder.record_output(&topic, data);
                    }
                }
            }

            let (tick_start, tick_duration, tick_result) = {
                let registered = &mut self.nodes[i];
                if let Some(ref mut context) = registered.context {
                    context.start_tick();

                    // Set node context for hlog!() macro
                    let tick_number = context.metrics().total_ticks();
                    set_node_context(&registered.name, tick_number);

                    // Execute node tick via NodeRunner (timing + panic isolation)
                    let tr = super::primitives::NodeRunner::run_tick(&mut registered.node);

                    clear_node_context();
                    (tr.tick_start, tr.duration, tr.result)
                } else {
                    return false;
                }
            };

            return self.process_tick_result(i, tick_start, tick_duration, tick_result);
        }
        false
    }

    /// Process the result of a node tick (profiling, budget, failure handling).
    /// Returns true if the scheduler should stop.
    fn process_tick_result(
        &mut self,
        i: usize,
        tick_start: Instant,
        tick_duration: Duration,
        tick_result: std::thread::Result<()>,
    ) -> bool {
        // Profiling and monitoring
        {
            let node_name = self.nodes[i].name.as_ref();
            let mut profiler = self.monitor.profiler.lock().unwrap();
            if tick_result.is_err() {
                profiler.record_node_failure(node_name);
                print_line(&format!("Node '{}' panicked during execution", node_name));
            }
            profiler.record(node_name, tick_duration);
        }

        // Update per-node RtStats
        if let Some(ref mut stats) = self.nodes[i].rt_stats {
            stats.record_execution(tick_duration);
        }

        // Capture outputs from publisher topics (for recording)
        {
            let RegisteredNode { ref mut node, ref mut recorder, .. } = self.nodes[i];
            if let Some(recorder) = recorder.as_mut() {
                if recorder.is_active_tick() {
                    let publishers = node.publishers();
                    if !publishers.is_empty() {
                        let topics_dir = crate::memory::platform::shm_topics_dir();
                        for pub_topic in &publishers {
                            let topic_path = topics_dir.join(&pub_topic.topic_name);
                            if let Some(slot_read) = crate::communication::read_latest_slot_bytes(&topic_path, 0) {
                                recorder.record_output(&pub_topic.topic_name, slot_read.payload);
                            }
                        }
                    }
                }
            }
        }

        // End recording tick
        if let Some(ref mut recorder) = self.nodes[i].recorder {
            recorder.end_tick(tick_duration.as_nanos() as u64);
        }

        // Check timing violations for RT nodes
        if self.check_timing_violations(i, tick_start, tick_duration) {
            return true; // Emergency stop triggered by deadline policy
        }

        // Handle tick result
        match tick_result {
            Ok(_) => {
                if let Some(ref mut context) = self.nodes[i].context {
                    context.record_tick();
                }

                // Recovery: successful tick transitions Warning→Healthy
                {
                    use super::types::NodeHealthState;
                    let health = self.nodes[i].health_state.load();
                    if health == NodeHealthState::Warning {
                        self.nodes[i].health_state.store(NodeHealthState::Healthy);
                        // Feed watchdog on recovery so the graduated check sees fresh timestamp
                        if let Some(ref monitor) = self.monitor.safety {
                            monitor.feed_watchdog(&self.nodes[i].name);
                        }
                    }
                }

                // Graduated degradation recovery: track successful ticks at reduced rate
                if let Some(ref monitor) = self.monitor.safety {
                    let action =
                        monitor.record_successful_tick(&self.nodes[i].name);
                    self.apply_degradation_action(i, action);
                }

                false
            }
            Err(panic_err) => self.handle_tick_failure(i, panic_err),
        }
    }

    /// Check tick budget and deadline violations for real-time nodes.
    ///
    /// Uses `TimingEnforcer` for the core budget/deadline detection, then dispatches
    /// node callbacks, stats updates, blackbox recording, and policy actions.
    ///
    /// Returns `true` if the scheduler should stop (EmergencyStop policy).
    fn check_timing_violations(
        &mut self,
        i: usize,
        tick_start: Instant,
        tick_duration: Duration,
    ) -> bool {
        use super::primitives::{DeadlineAction, TimingEnforcer};

        let node_name = Arc::clone(&self.nodes[i].name);

        // Check tick budget for RT nodes via TimingEnforcer
        if self.nodes[i].is_rt_node {
            if let Some(tick_budget) = self.nodes[i].tick_budget {
                if let Some(budget_result) =
                    TimingEnforcer::check_tick_budget(&node_name, tick_duration, tick_budget)
                {
                    let violation = &budget_result.violation;
                    print_line(&format!(
                        " budget violation in {}: {:?} > {:?}",
                        violation.node_name(), violation.actual(), violation.budget()
                    ));

                    // Update RtStats
                    if let Some(ref mut stats) = self.nodes[i].rt_stats {
                        stats.record_budget_violation();
                    }

                    // Record in blackbox
                    if let Some(ref bb) = self.monitor.blackbox {
                        bb.lock().unwrap().record(
                            super::blackbox::BlackBoxEvent::BudgetViolation {
                                name: node_name.to_string(),
                                budget_us: violation.budget().as_micros() as u64,
                                actual_us: violation.actual().as_micros() as u64,
                            },
                        );
                    }

                    // Also report to safety monitor if available
                    if let Some(ref monitor) = self.monitor.safety {
                        let _ = monitor.check_tick_budget(&node_name, tick_duration);
                    }
                }
            }
        }

        // Check deadline for RT nodes via TimingEnforcer
        if self.nodes[i].is_rt_node {
            if let Some(deadline) = self.nodes[i].deadline {
                if let Some(dm) =
                    TimingEnforcer::check_deadline(tick_start, deadline, self.nodes[i].miss_policy)
                {
                    if let Some(ref monitor) = self.monitor.safety {
                        monitor.record_deadline_miss(&node_name);
                    }
                    print_line(&format!(
                        " Deadline miss in {}: {:?} > {:?}",
                        node_name, dm.elapsed, dm.deadline
                    ));

                    // Update RtStats
                    if let Some(ref mut stats) = self.nodes[i].rt_stats {
                        stats.record_deadline_miss();
                    }

                    // Record in blackbox
                    if let Some(ref bb) = self.monitor.blackbox {
                        bb.lock()
                            .unwrap()
                            .record(super::blackbox::BlackBoxEvent::DeadlineMiss {
                                name: node_name.to_string(),
                                deadline_us: dm.deadline.as_micros() as u64,
                                actual_us: dm.elapsed.as_micros() as u64,
                            });
                    }

                    // Graduated degradation: evaluate and apply corrective action
                    if let Some(ref monitor) = self.monitor.safety {
                        let consecutive = monitor.consecutive_misses(&node_name);
                        let action = monitor.evaluate_degradation(
                            &node_name,
                            consecutive,
                            self.nodes[i].rate_hz,
                        );
                        self.apply_degradation_action(i, action);
                    }

                    // Dispatch on deadline action
                    match dm.action {
                        DeadlineAction::Warn => {
                            // Already logged above — no further action
                        }
                        DeadlineAction::Skip => {
                            self.nodes[i].is_paused = true;
                            print_line(&format!(
                                " Deadline policy: skipping '{}' for one tick",
                                node_name
                            ));
                        }
                        DeadlineAction::SafeMode => {
                            print_line(&format!(
                                " Deadline policy: '{}' entering safe state",
                                node_name
                            ));
                            self.nodes[i].node.enter_safe_state();
                            if let Some(ref monitor) = self.monitor.safety {
                                monitor.record_degrade_activation();
                            }
                        }
                        DeadlineAction::EmergencyStop => {
                            print_line(&format!(
                                " Deadline policy: emergency stop triggered by '{}'",
                                node_name
                            ));
                            if let Some(ref monitor) = self.monitor.safety {
                                monitor.trigger_emergency_stop(format!(
                                    "Deadline miss in '{}': {:?} > {:?}",
                                    node_name, dm.elapsed, dm.deadline
                                ));
                            }
                            return true;
                        }
                    }
                }
            }
        }
        false
    }

    /// Apply a degradation action produced by the safety monitor.
    fn apply_degradation_action(
        &mut self,
        i: usize,
        action: super::safety_monitor::DegradationAction,
    ) {
        use super::safety_monitor::DegradationAction;
        use super::types::NodeHealthState;

        match action {
            DegradationAction::None => {}
            DegradationAction::Warn(ref name) => {
                print_line(&format!(
                    " Degradation: '{}' — sustained timing violations, monitoring",
                    name
                ));
            }
            DegradationAction::ReduceRate { ref node, new_rate_hz } => {
                self.nodes[i].rate_hz = Some(new_rate_hz);
                self.nodes[i].last_tick = Some(Instant::now());
                print_line(&format!(
                    " Degradation: '{}' — reducing rate to {:.1} Hz",
                    node, new_rate_hz
                ));
            }
            DegradationAction::Isolate(ref name) => {
                self.nodes[i].health_state.store(NodeHealthState::Isolated);
                self.nodes[i].node.enter_safe_state();
                print_line(&format!(
                    " Degradation: '{}' — isolated, entered safe state",
                    name
                ));
                if let Some(ref monitor) = self.monitor.safety {
                    monitor.record_degrade_activation();
                }
            }
            DegradationAction::RestoreRate {
                ref node,
                original_rate_hz,
            } => {
                self.nodes[i].rate_hz = Some(original_rate_hz);
                self.nodes[i].last_tick = Some(Instant::now());
                self.nodes[i].health_state.store(NodeHealthState::Healthy);
                print_line(&format!(
                    " Recovery: '{}' — restored to {:.1} Hz",
                    node, original_rate_hz
                ));
            }
        }
    }

    /// Handle a node tick failure.
    /// Returns `true` if the scheduler should stop (fatal failure).
    fn handle_tick_failure(&mut self, i: usize, panic_err: Box<dyn std::any::Any + Send>) -> bool {
        let error_msg = if let Some(s) = panic_err.downcast_ref::<&str>() {
            format!("Node panicked: {}", s)
        } else if let Some(s) = panic_err.downcast_ref::<String>() {
            format!("Node panicked: {}", s)
        } else {
            "Node panicked with unknown error".to_string()
        };

        // Arc::clone — cheap atomic increment (only on node panic)
        let node_name = Arc::clone(&self.nodes[i].name);
        let registered = &mut self.nodes[i];
        if let Some(ref mut context) = registered.context {
            context.record_tick_failure(error_msg.clone());

            // Set context for on_error handler
            set_node_context(&node_name, context.metrics().total_ticks());
            registered.node.on_error(&error_msg);
            clear_node_context();

            print_line(&format!(
                " Node '{}' failed (continuing): {}",
                node_name, error_msg
            ));
            context.transition_to_error(error_msg);
        }
        false
    }

    /// Execute main-thread (BestEffort) nodes sequentially in priority order.
    ///
    /// RT, Compute, and Event nodes are handled by their dedicated executors.
    /// Only BestEffort nodes remain in `self.nodes` for main-thread execution.
    async fn execute_nodes(&mut self, node_filter: Option<&[&str]>) {
        // Sort by priority
        self.nodes.sort_by_key(|r| r.priority);

        let num_nodes = self.nodes.len();
        for i in 0..num_nodes {
            if self.execute_single_node(i, node_filter) {
                return; // Fatal failure — scheduler stopping
            }
        }
    }
}

#[cfg(test)]
mod tests;
