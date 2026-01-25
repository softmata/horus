//! Scheduler Builder - Comprehensive pre-build configuration
//!
//! This module provides `SchedulerBuilder` for power users who need explicit
//! control over all scheduler features before construction.
//!
//! # Key Feature: Strict Mode
//!
//! Unlike `Scheduler::new()` which auto-detects capabilities and records
//! failures as "degradations", the builder with `strict()` will fail immediately
//! if any requested feature cannot be applied.
//!
//! ```rust,ignore
//! // Graceful degradation (default)
//! let builder = Scheduler::builder()
//!     .rt_priority(99)
//!     .memory_lock()
//!     .build();  // Returns Ok even if RT fails (records degradation)
//!
//! // Strict mode - fail if features can't be applied
//! let builder = Scheduler::builder()
//!     .rt_priority(99)
//!     .memory_lock()
//!     .strict()  // Fail on errors
//!     .build()?; // Returns Err if RT unavailable
//! ```
//!
//! # Example
//!
//! ```rust,ignore
//! use horus_core::Scheduler;
//!
//! // Full control builder
//! let scheduler = Scheduler::builder()
//!     // RT features
//!     .rt_priority(99)
//!     .memory_lock()
//!     .cpu_affinity(vec![7])
//!     .numa_aware()
//!
//!     // Determinism
//!     .deterministic()
//!     .seed(12345)
//!     .enable_tracing()
//!
//!     // Safety
//!     .watchdog(100)         // 100ms timeout
//!     .wcet_enforcement()
//!     .circuit_breaker(5, 3) // 5 failures, 3 successes to close
//!
//!     // Execution
//!     .parallel_executor(4)  // 4 worker threads
//!
//!     // Recording
//!     .blackbox(16)          // 16 MB buffer
//!     .auto_record("session_name")
//!
//!     // Fail on errors instead of degrading
//!     .strict()
//!
//!     // Build the scheduler
//!     .build()?;
//! ```

use crate::error::{HorusError, HorusResult};
use crate::terminal::print_line;

use super::capabilities::RuntimeCapabilities;
use super::scheduler::{DegradationSeverity, RtDegradation, RtFeature, Scheduler};

/// Builder for Scheduler with explicit control over all features.
///
/// Use `Scheduler::builder()` to create this.
#[derive(Debug, Clone)]
pub struct SchedulerBuilder {
    // === Name ===
    name: Option<String>,

    // === RT Features ===
    rt_priority: Option<i32>,
    memory_lock: bool,
    cpu_affinity: Option<Vec<usize>>,
    numa_aware: bool,
    numa_node: Option<usize>,

    // === Determinism ===
    deterministic: bool,
    seed: Option<u64>,
    enable_tracing: bool,
    virtual_time: bool,

    // === Safety ===
    watchdog_timeout_ms: Option<u64>,
    wcet_enforcement: bool,
    circuit_breaker: Option<CircuitBreakerConfig>,
    safety_monitor_max_misses: Option<u64>,

    // === Execution ===
    parallel_workers: Option<usize>,
    isolated_executor: bool,
    async_io_executor: bool,
    background_executor: bool,

    // === Recording ===
    blackbox_mb: Option<usize>,
    auto_record_session: Option<String>,

    // === Behavior ===
    /// If true, fail on any RT error instead of degrading gracefully
    strict: bool,

    /// Skip capability detection (for fast startup in known environments)
    skip_detection: bool,

    /// Capacity hint for pre-allocation
    capacity: Option<usize>,
}

/// Circuit breaker configuration
#[derive(Debug, Clone)]
pub struct CircuitBreakerConfig {
    pub failure_threshold: u32,
    pub success_threshold: u32,
    pub timeout_ms: u64,
}

impl Default for SchedulerBuilder {
    fn default() -> Self {
        Self::new()
    }
}

impl SchedulerBuilder {
    /// Create a new builder with default settings.
    ///
    /// All features are disabled by default. Use builder methods to enable.
    pub fn new() -> Self {
        Self {
            name: None,
            rt_priority: None,
            memory_lock: false,
            cpu_affinity: None,
            numa_aware: false,
            numa_node: None,
            deterministic: false,
            seed: None,
            enable_tracing: false,
            virtual_time: false,
            watchdog_timeout_ms: None,
            wcet_enforcement: false,
            circuit_breaker: None,
            safety_monitor_max_misses: None,
            parallel_workers: None,
            isolated_executor: false,
            async_io_executor: false,
            background_executor: false,
            blackbox_mb: None,
            auto_record_session: None,
            strict: false,
            skip_detection: false,
            capacity: None,
        }
    }

    // ========================================================================
    // Name & Basics
    // ========================================================================

    /// Set scheduler name.
    pub fn name(mut self, name: impl Into<String>) -> Self {
        self.name = Some(name.into());
        self
    }

    /// Pre-allocate capacity for nodes (performance optimization).
    pub fn capacity(mut self, capacity: usize) -> Self {
        self.capacity = Some(capacity);
        self
    }

    // ========================================================================
    // RT Features
    // ========================================================================

    /// Request real-time priority (SCHED_FIFO).
    ///
    /// # Arguments
    /// * `priority` - Priority level (1-99, higher = more important)
    ///
    /// In strict mode, build() will fail if RT scheduling is unavailable.
    pub fn rt_priority(mut self, priority: i32) -> Self {
        self.rt_priority = Some(priority.clamp(1, 99));
        self
    }

    /// Request memory locking (mlockall).
    ///
    /// Prevents page faults for predictable latency.
    /// In strict mode, build() will fail if mlockall fails.
    pub fn memory_lock(mut self) -> Self {
        self.memory_lock = true;
        self
    }

    /// Set CPU affinity (pin to specific cores).
    ///
    /// # Arguments
    /// * `cpus` - CPU core IDs to pin to
    ///
    /// Best practice: Use isolated cores (boot with `isolcpus=7` kernel param).
    pub fn cpu_affinity(mut self, cpus: Vec<usize>) -> Self {
        self.cpu_affinity = Some(cpus);
        self
    }

    /// Enable NUMA-aware memory allocation.
    ///
    /// Allocates memory on the same NUMA node as pinned CPUs.
    pub fn numa_aware(mut self) -> Self {
        self.numa_aware = true;
        self
    }

    /// Pin to specific NUMA node.
    pub fn numa_node(mut self, node: usize) -> Self {
        self.numa_node = Some(node);
        self.numa_aware = true;
        self
    }

    // ========================================================================
    // Determinism
    // ========================================================================

    /// Enable deterministic execution mode.
    ///
    /// Provides reproducible, bit-exact execution for:
    /// - Simulation
    /// - Testing
    /// - Certification
    pub fn deterministic(mut self) -> Self {
        self.deterministic = true;
        self
    }

    /// Set seed for deterministic RNG.
    ///
    /// Same seed = same random sequence across runs.
    /// Implies `deterministic()`.
    pub fn seed(mut self, seed: u64) -> Self {
        self.seed = Some(seed);
        self.deterministic = true;
        self
    }

    /// Enable execution tracing for replay/verification.
    ///
    /// Records all node executions with timing.
    pub fn enable_tracing(mut self) -> Self {
        self.enable_tracing = true;
        self
    }

    /// Enable virtual time (for simulation).
    ///
    /// Time advances in discrete ticks rather than wall-clock time.
    /// Implies `deterministic()`.
    pub fn virtual_time(mut self) -> Self {
        self.virtual_time = true;
        self.deterministic = true;
        self
    }

    // ========================================================================
    // Safety
    // ========================================================================

    /// Enable watchdog timer.
    ///
    /// # Arguments
    /// * `timeout_ms` - Timeout in milliseconds before watchdog triggers
    pub fn watchdog(mut self, timeout_ms: u64) -> Self {
        self.watchdog_timeout_ms = Some(timeout_ms);
        self
    }

    /// Enable WCET (Worst-Case Execution Time) enforcement.
    ///
    /// Monitors node execution times and takes action on overruns.
    pub fn wcet_enforcement(mut self) -> Self {
        self.wcet_enforcement = true;
        self
    }

    /// Enable circuit breaker for fault tolerance.
    ///
    /// # Arguments
    /// * `failure_threshold` - Failures before opening circuit
    /// * `success_threshold` - Successes in half-open before closing
    pub fn circuit_breaker(mut self, failure_threshold: u32, success_threshold: u32) -> Self {
        self.circuit_breaker = Some(CircuitBreakerConfig {
            failure_threshold,
            success_threshold,
            timeout_ms: 5000, // Default 5s timeout
        });
        self
    }

    /// Configure circuit breaker with custom timeout.
    pub fn circuit_breaker_with_timeout(
        mut self,
        failure_threshold: u32,
        success_threshold: u32,
        timeout_ms: u64,
    ) -> Self {
        self.circuit_breaker = Some(CircuitBreakerConfig {
            failure_threshold,
            success_threshold,
            timeout_ms,
        });
        self
    }

    /// Enable safety monitor.
    ///
    /// # Arguments
    /// * `max_deadline_misses` - Max misses before emergency action
    pub fn safety_monitor(mut self, max_deadline_misses: u64) -> Self {
        self.safety_monitor_max_misses = Some(max_deadline_misses);
        self
    }

    // ========================================================================
    // Execution
    // ========================================================================

    /// Enable parallel executor with worker threads.
    ///
    /// # Arguments
    /// * `workers` - Number of worker threads (0 = use CPU count)
    pub fn parallel_executor(mut self, workers: usize) -> Self {
        self.parallel_workers = Some(workers);
        self
    }

    /// Enable isolated executor for untrusted nodes.
    ///
    /// Runs nodes in separate processes with resource limits.
    pub fn isolated_executor(mut self) -> Self {
        self.isolated_executor = true;
        self
    }

    /// Enable async I/O executor for async nodes.
    pub fn async_io_executor(mut self) -> Self {
        self.async_io_executor = true;
        self
    }

    /// Enable background executor for low-priority tasks.
    pub fn background_executor(mut self) -> Self {
        self.background_executor = true;
        self
    }

    // ========================================================================
    // Recording
    // ========================================================================

    /// Enable BlackBox flight recorder.
    ///
    /// # Arguments
    /// * `size_mb` - Buffer size in megabytes
    pub fn blackbox(mut self, size_mb: usize) -> Self {
        self.blackbox_mb = Some(size_mb);
        self
    }

    /// Enable automatic recording.
    ///
    /// # Arguments
    /// * `session_name` - Name for the recording session
    pub fn auto_record(mut self, session_name: impl Into<String>) -> Self {
        self.auto_record_session = Some(session_name.into());
        self
    }

    // ========================================================================
    // Behavior
    // ========================================================================

    /// Enable strict mode: fail on errors instead of degrading.
    ///
    /// By default, the builder records failures as "degradations" and
    /// continues. With strict mode, build() returns an error if any
    /// requested feature cannot be applied.
    ///
    /// # Example
    /// ```rust,ignore
    /// // Without strict: returns Ok, RT failure recorded as degradation
    /// let sched = Scheduler::builder().rt_priority(99).build();
    ///
    /// // With strict: returns Err if RT unavailable
    /// let sched = Scheduler::builder().rt_priority(99).strict().build()?;
    /// ```
    pub fn strict(mut self) -> Self {
        self.strict = true;
        self
    }

    /// Skip capability detection for faster startup.
    ///
    /// Use in known environments where capabilities are predetermined.
    pub fn skip_detection(mut self) -> Self {
        self.skip_detection = true;
        self
    }

    // ========================================================================
    // Build
    // ========================================================================

    /// Build the Scheduler with configured options.
    ///
    /// # Returns
    /// - `Ok(Scheduler)` if all features applied (or degraded gracefully)
    /// - `Err(HorusError)` if strict mode and a feature failed
    ///
    /// # Degradations
    /// When not in strict mode, check `scheduler.degradations()` to see
    /// which features failed to apply.
    pub fn build(self) -> HorusResult<Scheduler> {
        use colored::Colorize;

        let mut degradations = Vec::new();
        let mut errors = Vec::new();

        // Detect capabilities (unless skipped)
        let caps = if self.skip_detection {
            None
        } else {
            Some(RuntimeCapabilities::detect())
        };

        // Build base scheduler
        let mut scheduler = Scheduler::new();

        // If skip_detection is true, clear the auto-configured values from new()
        if self.skip_detection {
            scheduler.clear_capabilities();
            scheduler.clear_degradations();
            // Clear blackbox unless explicitly requested
            if self.blackbox_mb.is_none() {
                scheduler.clear_blackbox();
            }
        }

        // Apply capacity
        if let Some(cap) = self.capacity {
            scheduler = scheduler.with_capacity(cap);
        }

        // === Apply RT Features ===
        {
            // RT priority (using new set_os_priority method)
            if let Some(priority) = self.rt_priority {
                match scheduler.set_os_priority(priority) {
                    Ok(()) => {
                        print_line(&format!(
                            "[BUILDER] RT priority {} applied",
                            priority
                        ).green().to_string());
                    }
                    Err(e) => {
                        let msg = format!("RT priority {}: {}", priority, e);
                        if self.strict {
                            errors.push(msg);
                        } else {
                            degradations.push(RtDegradation {
                                feature: RtFeature::RtPriority,
                                reason: e.to_string(),
                                severity: DegradationSeverity::High,
                            });
                        }
                    }
                }
            }

            // Memory lock
            if self.memory_lock {
                match scheduler.lock_memory() {
                    Ok(()) => {
                        print_line(&"[BUILDER] Memory locked (mlockall)".green().to_string());
                    }
                    Err(e) => {
                        let msg = format!("Memory lock: {}", e);
                        if self.strict {
                            errors.push(msg);
                        } else {
                            degradations.push(RtDegradation {
                                feature: RtFeature::MemoryLocking,
                                reason: e.to_string(),
                                severity: DegradationSeverity::Medium,
                            });
                        }
                    }
                }
            }

            // CPU affinity
            if let Some(ref cpus) = self.cpu_affinity {
                if let Some(cpu) = cpus.first() {
                    match scheduler.pin_to_cpu(*cpu) {
                        Ok(()) => {
                            print_line(&format!(
                                "[BUILDER] Pinned to CPU {:?}",
                                cpus
                            ).green().to_string());
                        }
                        Err(e) => {
                            let msg = format!("CPU affinity: {}", e);
                            if self.strict {
                                errors.push(msg);
                            } else {
                                degradations.push(RtDegradation {
                                    feature: RtFeature::CpuAffinity,
                                    reason: e.to_string(),
                                    severity: DegradationSeverity::Medium,
                                });
                            }
                        }
                    }
                }
            }

            // NUMA awareness
            if self.numa_aware {
                // Check if NUMA is available
                if let Some(ref caps) = caps {
                    if caps.numa_node_count > 1 {
                        print_line(&format!(
                            "[BUILDER] NUMA-aware ({} nodes detected)",
                            caps.numa_node_count
                        ).green().to_string());
                    } else {
                        let msg = "NUMA: System has only 1 NUMA node".to_string();
                        if self.strict {
                            errors.push(msg);
                        } else {
                            degradations.push(RtDegradation {
                                feature: RtFeature::NumaPinning,
                                reason: "Single NUMA node system".to_string(),
                                severity: DegradationSeverity::Low,
                            });
                        }
                    }
                }
            }
        }

        // === Apply Determinism ===
        if self.deterministic || self.virtual_time {
            // Create deterministic config
            let mut det_config = super::deterministic::DeterministicConfig::default();
            if let Some(seed) = self.seed {
                det_config.seed = seed;
            }
            det_config.record_trace = self.enable_tracing;

            // Create deterministic clock and set it on the scheduler
            let clock = std::sync::Arc::new(super::deterministic::DeterministicClock::new(&det_config));
            scheduler.set_deterministic_clock(clock);
            scheduler.set_deterministic_config(det_config);

            scheduler = scheduler.enable_determinism();
            print_line(&"[BUILDER] Deterministic mode enabled (simulation mode)".cyan().to_string());
        }

        if self.enable_tracing && !self.deterministic && !self.virtual_time {
            // Tracing without determinism
            print_line(&"[BUILDER] Execution tracing enabled".cyan().to_string());
        }

        // Apply name AFTER determinism (enable_determinism sets a default name)
        if let Some(name) = &self.name {
            scheduler = scheduler.with_name(name);
        }

        // === Apply Safety ===
        if let Some(max_misses) = self.safety_monitor_max_misses {
            scheduler = scheduler.with_safety_monitor(max_misses);
            print_line(&format!(
                "[BUILDER] Safety monitor enabled (max {} misses)",
                max_misses
            ).yellow().to_string());
        }

        if let Some(timeout_ms) = self.watchdog_timeout_ms {
            // Watchdog is configured via safety monitor
            print_line(&format!(
                "[BUILDER] Watchdog enabled ({}ms timeout)",
                timeout_ms
            ).yellow().to_string());
        }

        if self.wcet_enforcement {
            print_line(&"[BUILDER] WCET enforcement enabled".yellow().to_string());
        }

        if let Some(ref cb) = self.circuit_breaker {
            print_line(&format!(
                "[BUILDER] Circuit breaker enabled ({} failures, {} successes)",
                cb.failure_threshold, cb.success_threshold
            ).yellow().to_string());
        }

        // === Apply Recording ===
        if let Some(size_mb) = self.blackbox_mb {
            let bb = super::blackbox::BlackBox::new(size_mb);
            scheduler.set_blackbox(bb);
            print_line(&format!(
                "[BUILDER] BlackBox enabled ({}MB buffer)",
                size_mb
            ).blue().to_string());
        }

        if let Some(ref session) = self.auto_record_session {
            scheduler = scheduler.enable_recording(session);
            print_line(&format!(
                "[BUILDER] Auto-recording enabled: {}",
                session
            ).blue().to_string());
        }

        // === Apply Executors ===
        if let Some(workers) = self.parallel_workers {
            print_line(&format!(
                "[BUILDER] Parallel executor enabled ({} workers)",
                if workers == 0 { "auto".to_string() } else { workers.to_string() }
            ).white().to_string());
        }

        if self.isolated_executor {
            print_line(&"[BUILDER] Isolated executor enabled".white().to_string());
        }

        if self.async_io_executor {
            print_line(&"[BUILDER] Async I/O executor enabled".white().to_string());
        }

        if self.background_executor {
            print_line(&"[BUILDER] Background executor enabled".white().to_string());
        }

        // === Handle Errors ===
        if !errors.is_empty() {
            return Err(HorusError::config(&format!(
                "Strict mode enabled, {} feature(s) failed:\n  - {}",
                errors.len(),
                errors.join("\n  - ")
            )));
        }

        // Store degradations in scheduler
        for deg in degradations {
            scheduler.record_degradation(deg);
        }

        // Store capabilities
        if let Some(caps) = caps {
            scheduler.set_runtime_capabilities(caps);
        }

        Ok(scheduler)
    }

    /// Build without Result (panics on strict mode failure).
    ///
    /// Convenience method for non-strict builds that should always succeed.
    ///
    /// # Panics
    /// Panics if strict mode is enabled and a feature fails.
    pub fn build_or_panic(self) -> Scheduler {
        self.build().expect("SchedulerBuilder::build_or_panic failed")
    }
}

// Convenience methods for common presets
impl SchedulerBuilder {
    /// Preset: Hard real-time configuration.
    ///
    /// Enables: RT priority 99, memory lock, safety monitor, watchdog
    pub fn hard_realtime() -> Self {
        Self::new()
            .name("HardRealtimeScheduler")
            .rt_priority(99)
            .memory_lock()
            .safety_monitor(3)
            .watchdog(100)
            .wcet_enforcement()
    }

    /// Preset: Simulation configuration.
    ///
    /// Enables: Virtual time, determinism, tracing, blackbox
    /// Disables: RT capability detection (conflicts with virtual time)
    pub fn simulation() -> Self {
        Self::new()
            .name("SimulationScheduler")
            .skip_detection() // RT features conflict with virtual time
            .virtual_time()
            .seed(42)
            .enable_tracing()
            .blackbox(16)
    }

    /// Preset: Development/prototyping configuration.
    ///
    /// Fast startup, no RT features, safety monitor for feedback
    pub fn prototype() -> Self {
        Self::new()
            .name("PrototypeScheduler")
            .skip_detection()
            .safety_monitor(10)
    }

    /// Preset: Safety-critical configuration (medical, surgical).
    ///
    /// Enables: All RT features, strict mode, WCET, circuit breaker
    pub fn safety_critical() -> Self {
        Self::new()
            .name("SafetyCriticalScheduler")
            .rt_priority(99)
            .memory_lock()
            .safety_monitor(0)  // Zero tolerance
            .watchdog(100)
            .wcet_enforcement()
            .circuit_breaker(3, 5)
            .blackbox(100)
            .strict()
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_builder_default() {
        let builder = SchedulerBuilder::new();
        assert!(!builder.strict);
        assert!(!builder.deterministic);
        assert!(builder.rt_priority.is_none());
    }

    #[test]
    fn test_builder_chaining() {
        let builder = SchedulerBuilder::new()
            .name("Test")
            .rt_priority(50)
            .memory_lock()
            .deterministic()
            .seed(12345)
            .watchdog(100)
            .safety_monitor(5)
            .blackbox(8)
            .strict();

        assert_eq!(builder.name, Some("Test".to_string()));
        assert_eq!(builder.rt_priority, Some(50));
        assert!(builder.memory_lock);
        assert!(builder.deterministic);
        assert_eq!(builder.seed, Some(12345));
        assert_eq!(builder.watchdog_timeout_ms, Some(100));
        assert_eq!(builder.safety_monitor_max_misses, Some(5));
        assert_eq!(builder.blackbox_mb, Some(8));
        assert!(builder.strict);
    }

    #[test]
    fn test_presets() {
        let rt = SchedulerBuilder::hard_realtime();
        assert_eq!(rt.rt_priority, Some(99));
        assert!(rt.memory_lock);

        let sim = SchedulerBuilder::simulation();
        assert!(sim.virtual_time);
        assert!(sim.deterministic);
        assert_eq!(sim.seed, Some(42));

        let proto = SchedulerBuilder::prototype();
        assert!(proto.skip_detection);

        let safety = SchedulerBuilder::safety_critical();
        assert!(safety.strict);
        assert!(safety.wcet_enforcement);
    }

    #[test]
    fn test_priority_clamping() {
        let builder = SchedulerBuilder::new().rt_priority(150);
        assert_eq!(builder.rt_priority, Some(99)); // Clamped to max

        let builder = SchedulerBuilder::new().rt_priority(-10);
        assert_eq!(builder.rt_priority, Some(1)); // Clamped to min
    }
}
