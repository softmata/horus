use crate::config::PySchedulerConfig;
use crate::node::PyNodeInfo;
use horus::memory::shm_heartbeats_dir;
use horus::{NodeHeartbeat, NodeInfo as CoreNodeInfo};
use pyo3::exceptions::PyRuntimeError;
use pyo3::prelude::*;
use pyo3::types::PyDict;
use std::fs;
use std::path::PathBuf;
use std::sync::{Arc, Mutex};
use std::thread;
use std::time::{Duration, Instant};

/// Registered node with priority, logging, and per-node rate control
struct RegisteredNode {
    node: PyObject,
    name: String,
    priority: u32,
    logging_enabled: bool,
    context: Arc<Mutex<CoreNodeInfo>>,
    cached_info: Option<Py<PyNodeInfo>>, // Cache PyNodeInfo to avoid creating new ones every tick
    rate_hz: f64,                        // Phase 1: Per-node rate control
    last_tick: Instant,                  // Phase 1: Track last execution time
    // Fault tolerance fields
    failure_count: u32,                    // Total failures
    consecutive_failures: u32,             // Consecutive failures (reset on success)
    circuit_open: bool,                    // Circuit breaker state
    last_restart_attempt: Option<Instant>, // Track when we last tried to restart
    // Soft real-time fields
    deadline_ms: Option<f64>,   // Optional deadline in milliseconds
    deadline_misses: u64,       // Counter for deadline violations
    last_tick_duration_ms: f64, // Last tick execution time
    // Watchdog fields
    watchdog_enabled: bool,      // Is watchdog enabled for this node?
    watchdog_timeout_ms: u64,    // Watchdog timeout in milliseconds
    last_watchdog_feed: Instant, // Last time watchdog was fed
    watchdog_expired: bool,      // Has watchdog expired?
    // Pub/Sub tracking for monitor
    publishers: Vec<String>,  // Topics this node publishes to
    subscribers: Vec<String>, // Topics this node subscribes to
}

/// Python wrapper for HORUS Scheduler with per-node rate control
///
/// The scheduler manages the execution of multiple nodes,
/// handling their lifecycle and coordinating their execution.
/// Supports per-node rate control for flexible scheduling.
#[pyclass(module = "horus._horus")]
pub struct PyScheduler {
    nodes: Arc<Mutex<Vec<RegisteredNode>>>,
    running: Arc<Mutex<bool>>,
    tick_rate_hz: f64,             // Global scheduler tick rate
    scheduler_name: String,        // Scheduler name for registry
    working_dir: PathBuf,          // Working directory for registry
    circuit_breaker_enabled: bool, // Fault tolerance: circuit breaker
    max_failures: u32,             // Max failures before circuit opens
    auto_restart: bool,            // Auto-restart failed nodes
    deadline_monitoring: bool,     // Soft real-time: enable deadline warnings
    watchdog_enabled: bool,        // Global watchdog enable flag
    watchdog_timeout_ms: u64,      // Default watchdog timeout
}

#[pymethods]
impl PyScheduler {
    #[new]
    #[pyo3(signature = (config=None))]
    pub fn new(config: Option<PySchedulerConfig>) -> PyResult<Self> {
        // Create heartbeat directory for monitor
        Self::setup_heartbeat_directory();

        // Extract config values or use defaults
        let (
            tick_rate,
            circuit_breaker,
            max_failures,
            auto_restart,
            deadline_monitoring,
            watchdog_enabled,
            watchdog_timeout_ms,
        ) = if let Some(ref cfg) = config {
            (
                cfg.tick_rate,
                cfg.circuit_breaker,
                cfg.max_failures,
                cfg.auto_restart,
                cfg.deadline_monitoring,
                cfg.watchdog_enabled,
                cfg.watchdog_timeout_ms,
            )
        } else {
            (100.0, true, 5, true, false, false, 1000) // Standard defaults
        };

        Ok(PyScheduler {
            nodes: Arc::new(Mutex::new(Vec::new())),
            running: Arc::new(Mutex::new(false)),
            tick_rate_hz: tick_rate,
            scheduler_name: "PythonScheduler".to_string(),
            working_dir: std::env::current_dir().unwrap_or_else(|_| PathBuf::from("/")),
            circuit_breaker_enabled: circuit_breaker,
            max_failures,
            auto_restart,
            deadline_monitoring,
            watchdog_enabled,
            watchdog_timeout_ms,
        })
    }

    /// Create scheduler from a configuration preset
    ///
    /// Example:
    ///     config = horus.SchedulerConfig.mobile()
    ///     scheduler = horus.Scheduler.from_config(config)
    #[staticmethod]
    pub fn from_config(config: PySchedulerConfig) -> PyResult<Self> {
        Self::new(Some(config))
    }

    /// Add a node with priority, logging, and optional rate control
    #[pyo3(signature = (node, priority, logging_enabled, rate_hz=None))]
    fn add(
        &mut self,
        py: Python,
        node: PyObject,
        priority: u32,
        logging_enabled: bool,
        rate_hz: Option<f64>,
    ) -> PyResult<()> {
        // Extract node name
        let name: String = node.getattr(py, "name")?.extract(py)?;

        // Extract publishers and subscribers from Python node
        let publishers: Vec<String> = node
            .getattr(py, "pub_topics")
            .and_then(|attr| attr.extract(py))
            .unwrap_or_default();
        let subscribers: Vec<String> = node
            .getattr(py, "sub_topics")
            .and_then(|attr| attr.extract(py))
            .unwrap_or_default();

        // Create NodeInfo context for this node
        let context = Arc::new(Mutex::new(CoreNodeInfo::new(name.clone(), logging_enabled)));

        // Use provided rate or default to global scheduler rate
        let node_rate = rate_hz.unwrap_or(self.tick_rate_hz);

        // Store the registered node
        let mut nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        nodes.push(RegisteredNode {
            node,
            name: name.clone(),
            priority,
            logging_enabled,
            context,
            cached_info: None,         // Will be created on first use
            rate_hz: node_rate,        // Phase 1: Per-node rate
            last_tick: Instant::now(), // Phase 1: Initialize timestamp
            // Fault tolerance fields
            failure_count: 0,
            consecutive_failures: 0,
            circuit_open: false,
            last_restart_attempt: None,
            // Soft real-time fields
            deadline_ms: None, // No deadline by default
            deadline_misses: 0,
            last_tick_duration_ms: 0.0,
            // Watchdog fields
            watchdog_enabled: false, // Disabled by default, enable per-node
            watchdog_timeout_ms: self.watchdog_timeout_ms, // Use global default
            last_watchdog_feed: Instant::now(),
            watchdog_expired: false,
            // Pub/Sub tracking
            publishers,
            subscribers,
        });

        println!(
            "Added node '{}' with priority {} (logging: {}, rate: {}Hz)",
            name, priority, logging_enabled, node_rate
        );

        Ok(())
    }

    /// Phase 1: Set per-node rate control
    fn set_node_rate(&mut self, node_name: String, rate_hz: f64) -> PyResult<()> {
        if rate_hz <= 0.0 || rate_hz > 10000.0 {
            return Err(PyRuntimeError::new_err(
                "Rate must be between 0 and 10000 Hz",
            ));
        }

        let mut nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        for registered in nodes.iter_mut() {
            if registered.name == node_name {
                registered.rate_hz = rate_hz;
                println!("Set node '{}' rate to {}Hz", node_name, rate_hz);
                return Ok(());
            }
        }

        Err(PyRuntimeError::new_err(format!(
            "Node '{}' not found",
            node_name
        )))
    }

    /// Set per-node deadline for soft real-time scheduling
    ///
    /// Args:
    ///     node_name: Name of the node
    ///     deadline_ms: Deadline in milliseconds (None to disable)
    ///
    /// The deadline is the maximum allowed execution time for a single tick.
    /// If a tick takes longer than the deadline, it's counted as a deadline miss.
    #[pyo3(signature = (node_name, deadline_ms=None))]
    fn set_node_deadline(&mut self, node_name: String, deadline_ms: Option<f64>) -> PyResult<()> {
        if let Some(d) = deadline_ms {
            if d <= 0.0 || d > 10000.0 {
                return Err(PyRuntimeError::new_err(
                    "Deadline must be between 0 and 10000 ms",
                ));
            }
        }

        let mut nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        for registered in nodes.iter_mut() {
            if registered.name == node_name {
                registered.deadline_ms = deadline_ms;
                if let Some(d) = deadline_ms {
                    println!("Set node '{}' deadline to {}ms", node_name, d);
                } else {
                    println!("Cleared deadline for node '{}'", node_name);
                }
                return Ok(());
            }
        }

        Err(PyRuntimeError::new_err(format!(
            "Node '{}' not found",
            node_name
        )))
    }

    /// Enable/disable watchdog timer for a specific node
    ///
    /// Args:
    ///     node_name: Name of the node
    ///     enabled: Enable or disable watchdog
    ///     timeout_ms: Optional timeout in milliseconds (uses global default if None)
    ///
    /// When enabled, the watchdog will be automatically fed on successful ticks.
    /// If the node fails to tick within the timeout, the watchdog expires.
    #[pyo3(signature = (node_name, enabled, timeout_ms=None))]
    fn set_node_watchdog(
        &mut self,
        node_name: String,
        enabled: bool,
        timeout_ms: Option<u64>,
    ) -> PyResult<()> {
        let timeout = timeout_ms.unwrap_or(self.watchdog_timeout_ms);

        if !(10..=60000).contains(&timeout) {
            return Err(PyRuntimeError::new_err(
                "Watchdog timeout must be between 10 and 60000 ms",
            ));
        }

        let mut nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        for registered in nodes.iter_mut() {
            if registered.name == node_name {
                registered.watchdog_enabled = enabled;
                registered.watchdog_timeout_ms = timeout;
                registered.last_watchdog_feed = Instant::now();
                registered.watchdog_expired = false;

                if enabled {
                    println!(
                        "Enabled watchdog for node '{}' with {}ms timeout",
                        node_name, timeout
                    );
                } else {
                    println!("Disabled watchdog for node '{}'", node_name);
                }
                return Ok(());
            }
        }

        Err(PyRuntimeError::new_err(format!(
            "Node '{}' not found",
            node_name
        )))
    }

    /// Phase 1: Get node statistics
    fn get_node_stats(&self, py: Python, node_name: String) -> PyResult<PyObject> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        for registered in nodes.iter() {
            if registered.name == node_name {
                let dict = PyDict::new(py);
                dict.set_item("name", &registered.name)?;
                dict.set_item("priority", registered.priority)?;
                dict.set_item("rate_hz", registered.rate_hz)?;
                dict.set_item("logging_enabled", registered.logging_enabled)?;

                // Fault tolerance info
                dict.set_item("failure_count", registered.failure_count)?;
                dict.set_item("consecutive_failures", registered.consecutive_failures)?;
                dict.set_item("circuit_open", registered.circuit_open)?;

                // Soft real-time info
                if let Some(deadline) = registered.deadline_ms {
                    dict.set_item("deadline_ms", deadline)?;
                } else {
                    dict.set_item("deadline_ms", py.None())?;
                }
                dict.set_item("deadline_misses", registered.deadline_misses)?;
                dict.set_item("last_tick_duration_ms", registered.last_tick_duration_ms)?;

                // Watchdog info
                dict.set_item("watchdog_enabled", registered.watchdog_enabled)?;
                dict.set_item("watchdog_timeout_ms", registered.watchdog_timeout_ms)?;
                dict.set_item("watchdog_expired", registered.watchdog_expired)?;
                if registered.watchdog_enabled {
                    let time_since_feed =
                        registered.last_watchdog_feed.elapsed().as_millis() as u64;
                    dict.set_item("watchdog_time_since_feed_ms", time_since_feed)?;
                } else {
                    dict.set_item("watchdog_time_since_feed_ms", py.None())?;
                }

                // Get metrics from NodeInfo
                if let Ok(ctx) = registered.context.lock() {
                    let metrics = ctx.metrics();
                    dict.set_item("total_ticks", metrics.total_ticks)?;
                    dict.set_item("successful_ticks", metrics.successful_ticks)?;
                    dict.set_item("failed_ticks", metrics.failed_ticks)?;
                    dict.set_item("errors_count", metrics.errors_count)?;
                    dict.set_item("avg_tick_duration_ms", metrics.avg_tick_duration_ms)?;
                    dict.set_item("min_tick_duration_ms", metrics.min_tick_duration_ms)?;
                    dict.set_item("max_tick_duration_ms", metrics.max_tick_duration_ms)?;
                    dict.set_item("last_tick_duration_ms", metrics.last_tick_duration_ms)?;
                    dict.set_item("uptime_seconds", ctx.uptime().as_secs_f64())?;
                    dict.set_item("state", format!("{}", ctx.state()))?;
                }

                return Ok(dict.into());
            }
        }

        Err(PyRuntimeError::new_err(format!(
            "Node '{}' not found",
            node_name
        )))
    }

    /// Remove a node from the scheduler
    fn remove_node(&mut self, name: String) -> PyResult<bool> {
        let mut nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        let original_len = nodes.len();
        nodes.retain(|n| n.name != name);
        Ok(nodes.len() < original_len)
    }

    /// Set the tick rate in Hz
    fn set_tick_rate(&mut self, rate_hz: f64) -> PyResult<()> {
        if rate_hz <= 0.0 || rate_hz > 10000.0 {
            return Err(PyRuntimeError::new_err(
                "Tick rate must be between 0 and 10000 Hz",
            ));
        }
        self.tick_rate_hz = rate_hz;
        Ok(())
    }

    /// Run the scheduler for a specified duration (in seconds)
    fn run_for(&mut self, py: Python, duration_seconds: f64) -> PyResult<()> {
        if duration_seconds <= 0.0 {
            return Err(PyRuntimeError::new_err("Duration must be positive"));
        }

        let tick_duration = Duration::from_secs_f64(1.0 / self.tick_rate_hz);
        let total_ticks = (duration_seconds * self.tick_rate_hz) as usize;

        // Set running flag
        {
            let mut running = self.running.lock().map_err(|e| {
                PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
            })?;
            *running = true;
        }

        // Initialize all nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                let py_info = Py::new(
                    py,
                    PyNodeInfo {
                        inner: registered.context.clone(),
                        scheduler_running: Some(self.running.clone()),
                    },
                )?;

                // Try calling with NodeInfo parameter first, fallback to no-arg version
                let result = registered
                    .node
                    .call_method1(py, "init", (py_info,))
                    .or_else(|_| registered.node.call_method0(py, "init"));

                if let Err(e) = result {
                    eprintln!("Failed to initialize node '{}': {:?}", registered.name, e);
                }
            }

            // Write initial registry for monitor
            Self::update_registry(&nodes, &self.scheduler_name, &self.working_dir);
        }

        // Setup heartbeat directory
        Self::setup_heartbeat_directory();

        // Track last snapshot time
        let mut last_snapshot = std::time::Instant::now();

        // Main execution loop
        for tick in 0..total_ticks {
            let tick_start = std::time::Instant::now();

            // Check if we should stop
            {
                let running = self.running.lock().map_err(|e| {
                    PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
                })?;
                if !*running {
                    break;
                }
            }

            // Execute tick for all nodes in priority order
            {
                let mut nodes = self
                    .nodes
                    .lock()
                    .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

                // Sort by priority (lower number = higher priority)
                nodes.sort_by_key(|r| r.priority);

                for registered in nodes.iter_mut() {
                    // Phase 1: Check if enough time has elapsed for this node's rate
                    let now = Instant::now();
                    let elapsed_secs = (now - registered.last_tick).as_secs_f64();
                    let period_secs = 1.0 / registered.rate_hz;

                    // Skip this node if not enough time has passed
                    if elapsed_secs < period_secs {
                        continue;
                    }

                    // Update last tick time
                    registered.last_tick = now;

                    // Check watchdog expiration
                    if registered.watchdog_enabled && self.watchdog_enabled {
                        let time_since_feed =
                            registered.last_watchdog_feed.elapsed().as_millis() as u64;
                        if time_since_feed > registered.watchdog_timeout_ms
                            && !registered.watchdog_expired
                        {
                            registered.watchdog_expired = true;
                            use colored::Colorize;
                            eprintln!(
                                "{}",
                                format!(
                                    "Watchdog expired: node '{}' not fed for {}ms (timeout: {}ms)",
                                    registered.name,
                                    time_since_feed,
                                    registered.watchdog_timeout_ms
                                )
                                .red()
                            );
                        }
                    }

                    // Start tick timing
                    if let Ok(mut ctx) = registered.context.lock() {
                        ctx.start_tick();
                    }

                    // Get or create cached PyNodeInfo
                    let py_info = if let Some(ref cached) = registered.cached_info {
                        cached.clone_ref(py)
                    } else {
                        let new_info = Py::new(
                            py,
                            PyNodeInfo {
                                inner: registered.context.clone(),
                                scheduler_running: Some(self.running.clone()),
                            },
                        )?;
                        registered.cached_info = Some(new_info.clone_ref(py));
                        new_info
                    };

                    // Measure tick start time
                    let tick_start = Instant::now();

                    // Try calling with NodeInfo parameter first, fallback to no-arg version
                    let result = registered
                        .node
                        .call_method1(py, "tick", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "tick"));

                    // Measure tick duration
                    let tick_duration = tick_start.elapsed();
                    let tick_duration_ms = tick_duration.as_secs_f64() * 1000.0;
                    registered.last_tick_duration_ms = tick_duration_ms;

                    // Check deadline
                    if let Some(deadline_ms) = registered.deadline_ms {
                        if tick_duration_ms > deadline_ms {
                            registered.deadline_misses += 1;
                            if self.deadline_monitoring {
                                use colored::Colorize;
                                eprintln!(
                                    "{}",
                                    format!(
                                        "Deadline miss: node '{}' took {:.3}ms (deadline: {}ms, miss #{})",
                                        registered.name,
                                        tick_duration_ms,
                                        deadline_ms,
                                        registered.deadline_misses
                                    )
                                    .yellow()
                                );
                            }
                        }
                    }

                    match result {
                        Ok(_) => {
                            // Success - reset consecutive failures
                            registered.consecutive_failures = 0;

                            // Feed watchdog on successful tick
                            if registered.watchdog_enabled {
                                registered.last_watchdog_feed = Instant::now();
                                registered.watchdog_expired = false;
                            }

                            // Record tick completion
                            if let Ok(mut ctx) = registered.context.lock() {
                                ctx.record_tick();
                            }
                        }
                        Err(e) => {
                            // Check if this is a KeyboardInterrupt - if so, stop the scheduler immediately
                            if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                                use colored::Colorize;
                                eprintln!(
                                    "{}",
                                    "\nKeyboard interrupt received, shutting down...".red()
                                );
                                if let Ok(mut r) = self.running.lock() {
                                    *r = false;
                                }
                                break;
                            }

                            // Track failures
                            registered.failure_count += 1;
                            registered.consecutive_failures += 1;

                            use colored::Colorize;
                            eprintln!(
                                "{}",
                                format!(
                                    "Error in node '{}' tick (failure {}/{}): {:?}",
                                    registered.name,
                                    registered.consecutive_failures,
                                    self.max_failures,
                                    e
                                )
                                .red()
                            );

                            // Check if we should open the circuit breaker
                            if self.circuit_breaker_enabled
                                && registered.consecutive_failures >= self.max_failures
                            {
                                registered.circuit_open = true;
                                eprintln!(
                                    "{}",
                                    format!(
                                        "Circuit breaker opened for node '{}' after {} consecutive failures",
                                        registered.name, registered.consecutive_failures
                                    )
                                    .yellow()
                                    .bold()
                                );
                            }
                        }
                    }

                    // Write heartbeat for monitor
                    if let Ok(ctx) = registered.context.lock() {
                        Self::write_heartbeat(&registered.name, &ctx, registered.rate_hz);
                    }
                }
            }

            // Periodic registry snapshot (every 5 seconds)
            if last_snapshot.elapsed() >= Duration::from_secs(5) {
                let nodes = self
                    .nodes
                    .lock()
                    .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
                Self::snapshot_state_to_registry(&nodes, &self.scheduler_name, &self.working_dir);
                last_snapshot = std::time::Instant::now();
            }

            // Sleep for remainder of tick period
            let elapsed = tick_start.elapsed();
            if elapsed < tick_duration {
                thread::sleep(tick_duration - elapsed);
            } else if tick % 100 == 0 {
                // Warn about timing issues every 100 ticks
                eprintln!(
                    "Warning: Tick {} took {:?}, longer than period {:?}",
                    tick, elapsed, tick_duration
                );
            }
        }

        // Clean up registry, heartbeats and session
        Self::cleanup_registry();
        Self::cleanup_heartbeats();
        Self::cleanup_session();

        // Shutdown all nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                let py_info = Py::new(
                    py,
                    PyNodeInfo {
                        inner: registered.context.clone(),
                        scheduler_running: Some(self.running.clone()),
                    },
                )?;

                // Try calling with NodeInfo parameter first, fallback to no-arg version
                let result = registered
                    .node
                    .call_method1(py, "shutdown", (py_info,))
                    .or_else(|_| registered.node.call_method0(py, "shutdown"));

                if let Err(e) = result {
                    eprintln!("Failed to shutdown node '{}': {:?}", registered.name, e);
                }
            }
        }

        // Clear running flag
        {
            let mut running = self.running.lock().map_err(|e| {
                PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
            })?;
            *running = false;
        }

        Ok(())
    }

    /// Run the scheduler indefinitely (until stop() is called)
    fn run(&mut self, py: Python) -> PyResult<()> {
        let tick_duration = Duration::from_secs_f64(1.0 / self.tick_rate_hz);

        // Set running flag
        {
            let mut running = self.running.lock().map_err(|e| {
                PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
            })?;
            *running = true;
        }

        // Set up Ctrl+C handler for immediate shutdown
        let running_clone = self.running.clone();
        let _ = ctrlc::set_handler(move || {
            eprintln!("\nCtrl+C received! Shutting down scheduler...");
            if let Ok(mut r) = running_clone.lock() {
                *r = false;
            }
        });

        // Initialize all nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                let py_info = Py::new(
                    py,
                    PyNodeInfo {
                        inner: registered.context.clone(),
                        scheduler_running: Some(self.running.clone()),
                    },
                )?;

                // Try calling with NodeInfo parameter first, fallback to no-arg version
                let result = registered
                    .node
                    .call_method1(py, "init", (py_info,))
                    .or_else(|_| registered.node.call_method0(py, "init"));

                if let Err(e) = result {
                    eprintln!("Failed to initialize node '{}': {:?}", registered.name, e);
                }
            }

            // Write initial registry for monitor
            Self::update_registry(&nodes, &self.scheduler_name, &self.working_dir);
        }

        // Setup heartbeat directory
        Self::setup_heartbeat_directory();

        // Track last snapshot time
        let mut last_snapshot = std::time::Instant::now();

        // Main execution loop
        loop {
            let tick_start = std::time::Instant::now();

            // Check for Python signals (e.g., Ctrl+C/KeyboardInterrupt)
            py.check_signals()?;

            // Check if we should stop
            {
                let running = self.running.lock().map_err(|e| {
                    PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
                })?;
                if !*running {
                    break;
                }
            }

            // Execute tick for all nodes in priority order
            {
                let mut nodes = self
                    .nodes
                    .lock()
                    .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

                // Sort by priority (lower number = higher priority)
                nodes.sort_by_key(|r| r.priority);

                for registered in nodes.iter_mut() {
                    // Skip nodes with open circuit breaker
                    if self.circuit_breaker_enabled && registered.circuit_open {
                        // Check if we should attempt auto-restart
                        if self.auto_restart {
                            let should_retry = match registered.last_restart_attempt {
                                None => true, // First restart attempt
                                Some(last_attempt) => {
                                    // Wait at least 5 seconds between restart attempts
                                    last_attempt.elapsed() >= Duration::from_secs(5)
                                }
                            };

                            if should_retry {
                                use colored::Colorize;
                                eprintln!(
                                    "{}",
                                    format!("Attempting to restart node '{}'...", registered.name)
                                        .yellow()
                                );
                                registered.circuit_open = false;
                                registered.consecutive_failures = 0;
                                registered.last_restart_attempt = Some(Instant::now());
                            }
                        }

                        if registered.circuit_open {
                            continue; // Still open, skip this node
                        }
                    }

                    // Phase 1: Check if enough time has elapsed for this node's rate
                    let now = Instant::now();
                    let elapsed_secs = (now - registered.last_tick).as_secs_f64();
                    let period_secs = 1.0 / registered.rate_hz;

                    // Skip this node if not enough time has passed
                    if elapsed_secs < period_secs {
                        continue;
                    }

                    // Update last tick time
                    registered.last_tick = now;

                    // Check watchdog expiration
                    if registered.watchdog_enabled && self.watchdog_enabled {
                        let time_since_feed =
                            registered.last_watchdog_feed.elapsed().as_millis() as u64;
                        if time_since_feed > registered.watchdog_timeout_ms
                            && !registered.watchdog_expired
                        {
                            registered.watchdog_expired = true;
                            use colored::Colorize;
                            eprintln!(
                                "{}",
                                format!(
                                    "Watchdog expired: node '{}' not fed for {}ms (timeout: {}ms)",
                                    registered.name,
                                    time_since_feed,
                                    registered.watchdog_timeout_ms
                                )
                                .red()
                            );
                        }
                    }

                    // Start tick timing
                    if let Ok(mut ctx) = registered.context.lock() {
                        ctx.start_tick();
                    }

                    // Get or create cached PyNodeInfo
                    let py_info = if let Some(ref cached) = registered.cached_info {
                        cached.clone_ref(py)
                    } else {
                        let new_info = Py::new(
                            py,
                            PyNodeInfo {
                                inner: registered.context.clone(),
                                scheduler_running: Some(self.running.clone()),
                            },
                        )?;
                        registered.cached_info = Some(new_info.clone_ref(py));
                        new_info
                    };

                    // Measure tick start time
                    let tick_start = Instant::now();

                    // Try calling with NodeInfo parameter first, fallback to no-arg version
                    let result = registered
                        .node
                        .call_method1(py, "tick", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "tick"));

                    // Measure tick duration
                    let tick_duration = tick_start.elapsed();
                    let tick_duration_ms = tick_duration.as_secs_f64() * 1000.0;
                    registered.last_tick_duration_ms = tick_duration_ms;

                    // Check deadline
                    if let Some(deadline_ms) = registered.deadline_ms {
                        if tick_duration_ms > deadline_ms {
                            registered.deadline_misses += 1;
                            if self.deadline_monitoring {
                                use colored::Colorize;
                                eprintln!(
                                    "{}",
                                    format!(
                                        "Deadline miss: node '{}' took {:.3}ms (deadline: {}ms, miss #{})",
                                        registered.name,
                                        tick_duration_ms,
                                        deadline_ms,
                                        registered.deadline_misses
                                    )
                                    .yellow()
                                );
                            }
                        }
                    }

                    match result {
                        Ok(_) => {
                            // Success - reset consecutive failures
                            registered.consecutive_failures = 0;

                            // Feed watchdog on successful tick
                            if registered.watchdog_enabled {
                                registered.last_watchdog_feed = Instant::now();
                                registered.watchdog_expired = false;
                            }

                            // Record tick completion
                            if let Ok(mut ctx) = registered.context.lock() {
                                ctx.record_tick();
                            }
                        }
                        Err(e) => {
                            // Check if this is a KeyboardInterrupt - if so, stop the scheduler immediately
                            if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                                use colored::Colorize;
                                eprintln!(
                                    "{}",
                                    "\nKeyboard interrupt received, shutting down...".red()
                                );
                                if let Ok(mut r) = self.running.lock() {
                                    *r = false;
                                }
                                break;
                            }

                            // Track failures
                            registered.failure_count += 1;
                            registered.consecutive_failures += 1;

                            use colored::Colorize;
                            eprintln!(
                                "{}",
                                format!(
                                    "Error in node '{}' tick (failure {}/{}): {:?}",
                                    registered.name,
                                    registered.consecutive_failures,
                                    self.max_failures,
                                    e
                                )
                                .red()
                            );

                            // Check if we should open the circuit breaker
                            if self.circuit_breaker_enabled
                                && registered.consecutive_failures >= self.max_failures
                            {
                                registered.circuit_open = true;
                                eprintln!(
                                    "{}",
                                    format!(
                                        "Circuit breaker opened for node '{}' after {} consecutive failures",
                                        registered.name, registered.consecutive_failures
                                    )
                                    .yellow()
                                    .bold()
                                );
                            }
                        }
                    }

                    // Write heartbeat for monitor
                    if let Ok(ctx) = registered.context.lock() {
                        Self::write_heartbeat(&registered.name, &ctx, registered.rate_hz);
                    }
                }
            }

            // Periodic registry snapshot (every 5 seconds)
            if last_snapshot.elapsed() >= Duration::from_secs(5) {
                let nodes = self
                    .nodes
                    .lock()
                    .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
                Self::snapshot_state_to_registry(&nodes, &self.scheduler_name, &self.working_dir);
                last_snapshot = std::time::Instant::now();
            }

            // Sleep for remainder of tick period
            let elapsed = tick_start.elapsed();
            if elapsed < tick_duration {
                thread::sleep(tick_duration - elapsed);
            }
        }

        // Clean up registry, heartbeats and session
        Self::cleanup_registry();
        Self::cleanup_heartbeats();
        Self::cleanup_session();

        // Shutdown all nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                let py_info = Py::new(
                    py,
                    PyNodeInfo {
                        inner: registered.context.clone(),
                        scheduler_running: Some(self.running.clone()),
                    },
                )?;

                // Try calling with NodeInfo parameter first, fallback to no-arg version
                let result = registered
                    .node
                    .call_method1(py, "shutdown", (py_info,))
                    .or_else(|_| registered.node.call_method0(py, "shutdown"));

                if let Err(e) = result {
                    eprintln!("Failed to shutdown node '{}': {:?}", registered.name, e);
                }
            }
        }

        Ok(())
    }

    /// Stop the scheduler
    fn stop(&mut self) -> PyResult<()> {
        let mut running = self
            .running
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e)))?;
        *running = false;
        Ok(())
    }

    /// Check if the scheduler is running
    fn is_running(&self) -> PyResult<bool> {
        let running = self
            .running
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e)))?;
        Ok(*running)
    }

    /// Get list of all nodes with basic information
    ///
    /// Returns a list of dictionaries containing node information:
    /// - name: Node name
    /// - priority: Execution priority
    /// - rate_hz: Node execution rate
    /// - logging_enabled: Whether logging is enabled
    /// - total_ticks: Total number of ticks executed
    /// - failure_count: Total failure count
    /// - consecutive_failures: Current consecutive failure count
    /// - circuit_open: Whether circuit breaker is open
    fn get_all_nodes(&self, py: Python) -> PyResult<Vec<PyObject>> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        let mut result = Vec::new();

        for registered in nodes.iter() {
            let dict = PyDict::new(py);
            dict.set_item("name", &registered.name)?;
            dict.set_item("priority", registered.priority)?;
            dict.set_item("rate_hz", registered.rate_hz)?;
            dict.set_item("logging_enabled", registered.logging_enabled)?;

            // Fault tolerance info
            dict.set_item("failure_count", registered.failure_count)?;
            dict.set_item("consecutive_failures", registered.consecutive_failures)?;
            dict.set_item("circuit_open", registered.circuit_open)?;

            // Soft real-time info
            if let Some(deadline) = registered.deadline_ms {
                dict.set_item("deadline_ms", deadline)?;
            } else {
                dict.set_item("deadline_ms", py.None())?;
            }
            dict.set_item("deadline_misses", registered.deadline_misses)?;
            dict.set_item("last_tick_duration_ms", registered.last_tick_duration_ms)?;

            // Watchdog info
            dict.set_item("watchdog_enabled", registered.watchdog_enabled)?;
            dict.set_item("watchdog_timeout_ms", registered.watchdog_timeout_ms)?;
            dict.set_item("watchdog_expired", registered.watchdog_expired)?;
            if registered.watchdog_enabled {
                let time_since_feed = registered.last_watchdog_feed.elapsed().as_millis() as u64;
                dict.set_item("watchdog_time_since_feed_ms", time_since_feed)?;
            } else {
                dict.set_item("watchdog_time_since_feed_ms", py.None())?;
            }

            // Get metrics from NodeInfo
            if let Ok(ctx) = registered.context.lock() {
                let metrics = ctx.metrics();
                dict.set_item("total_ticks", metrics.total_ticks)?;
                dict.set_item("successful_ticks", metrics.successful_ticks)?;
                dict.set_item("failed_ticks", metrics.failed_ticks)?;
                dict.set_item("errors_count", metrics.errors_count)?;
                dict.set_item("avg_tick_duration_ms", metrics.avg_tick_duration_ms)?;
                dict.set_item("uptime_seconds", ctx.uptime().as_secs_f64())?;
                dict.set_item("state", format!("{}", ctx.state()))?;
            }

            result.push(dict.into());
        }

        Ok(result)
    }

    /// Get count of all registered nodes
    fn get_node_count(&self) -> PyResult<usize> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
        Ok(nodes.len())
    }

    /// Check if a node exists by name
    fn has_node(&self, name: String) -> PyResult<bool> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
        Ok(nodes.iter().any(|n| n.name == name))
    }

    /// Get list of node names
    fn get_node_names(&self) -> PyResult<Vec<String>> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
        Ok(nodes.iter().map(|n| n.name.clone()).collect())
    }

    /// Run specific nodes by name (continuously until stop() is called)
    fn tick(&mut self, py: Python, node_names: Vec<String>) -> PyResult<()> {
        let tick_duration = Duration::from_secs_f64(1.0 / self.tick_rate_hz);

        // Set running flag
        {
            let mut running = self.running.lock().map_err(|e| {
                PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
            })?;
            *running = true;
        }

        // Initialize filtered nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                if node_names.contains(&registered.name) {
                    let py_info = Py::new(
                        py,
                        PyNodeInfo {
                            inner: registered.context.clone(),
                            scheduler_running: Some(self.running.clone()),
                        },
                    )?;

                    let result = registered
                        .node
                        .call_method1(py, "init", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "init"));

                    if let Err(e) = result {
                        eprintln!("Failed to initialize node '{}': {:?}", registered.name, e);
                    }
                }
            }
        }

        // Main execution loop
        loop {
            let tick_start = std::time::Instant::now();

            // Check for Python signals (e.g., Ctrl+C/KeyboardInterrupt)
            py.check_signals()?;

            // Check if we should stop
            {
                let running = self.running.lock().map_err(|e| {
                    PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
                })?;
                if !*running {
                    break;
                }
            }

            // Execute tick for filtered nodes in priority order
            {
                let mut nodes = self
                    .nodes
                    .lock()
                    .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

                nodes.sort_by_key(|r| r.priority);

                for registered in nodes.iter_mut() {
                    // Skip nodes not in the filter list
                    if !node_names.contains(&registered.name) {
                        continue;
                    }

                    // Check rate control
                    let now = Instant::now();
                    let elapsed_secs = (now - registered.last_tick).as_secs_f64();
                    let period_secs = 1.0 / registered.rate_hz;

                    if elapsed_secs < period_secs {
                        continue;
                    }

                    registered.last_tick = now;

                    // Start tick timing
                    if let Ok(mut ctx) = registered.context.lock() {
                        ctx.start_tick();
                    }

                    // Get or create cached PyNodeInfo
                    let py_info = if let Some(ref cached) = registered.cached_info {
                        cached.clone_ref(py)
                    } else {
                        let new_info = Py::new(
                            py,
                            PyNodeInfo {
                                inner: registered.context.clone(),
                                scheduler_running: Some(self.running.clone()),
                            },
                        )?;
                        registered.cached_info = Some(new_info.clone_ref(py));
                        new_info
                    };

                    let result = registered
                        .node
                        .call_method1(py, "tick", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "tick"));

                    if let Err(e) = result {
                        // Check if this is a KeyboardInterrupt - if so, stop the scheduler immediately
                        if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                            use colored::Colorize;
                            eprintln!(
                                "{}",
                                "\nKeyboard interrupt received, shutting down...".red()
                            );
                            if let Ok(mut r) = self.running.lock() {
                                *r = false;
                            }
                            break;
                        }
                        eprintln!("Error in node '{}' tick: {:?}", registered.name, e);
                    }

                    // Record tick completion
                    if let Ok(mut ctx) = registered.context.lock() {
                        ctx.record_tick();
                    }

                    // Write heartbeat for monitor
                    if let Ok(ctx) = registered.context.lock() {
                        Self::write_heartbeat(&registered.name, &ctx, registered.rate_hz);
                    }
                }
            }

            // Sleep for remainder of tick period
            let elapsed = tick_start.elapsed();
            if elapsed < tick_duration {
                thread::sleep(tick_duration - elapsed);
            }
        }

        // Shutdown filtered nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                if node_names.contains(&registered.name) {
                    let py_info = Py::new(
                        py,
                        PyNodeInfo {
                            inner: registered.context.clone(),
                            scheduler_running: Some(self.running.clone()),
                        },
                    )?;

                    let result = registered
                        .node
                        .call_method1(py, "shutdown", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "shutdown"));

                    if let Err(e) = result {
                        eprintln!("Failed to shutdown node '{}': {:?}", registered.name, e);
                    }
                }
            }
        }

        Ok(())
    }

    /// Run specific nodes for a specified duration (in seconds)
    fn tick_for(
        &mut self,
        py: Python,
        node_names: Vec<String>,
        duration_seconds: f64,
    ) -> PyResult<()> {
        if duration_seconds <= 0.0 {
            return Err(PyRuntimeError::new_err("Duration must be positive"));
        }

        let tick_duration = Duration::from_secs_f64(1.0 / self.tick_rate_hz);
        let start_time = Instant::now();
        let max_duration = Duration::from_secs_f64(duration_seconds);

        // Set running flag
        {
            let mut running = self.running.lock().map_err(|e| {
                PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
            })?;
            *running = true;
        }

        // Initialize filtered nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                if node_names.contains(&registered.name) {
                    let py_info = Py::new(
                        py,
                        PyNodeInfo {
                            inner: registered.context.clone(),
                            scheduler_running: Some(self.running.clone()),
                        },
                    )?;

                    let result = registered
                        .node
                        .call_method1(py, "init", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "init"));

                    if let Err(e) = result {
                        eprintln!("Failed to initialize node '{}': {:?}", registered.name, e);
                    }
                }
            }
        }

        // Main execution loop with time limit
        loop {
            let tick_start = std::time::Instant::now();

            // Check for Python signals (e.g., Ctrl+C/KeyboardInterrupt)
            py.check_signals()?;

            // Check if duration exceeded
            if start_time.elapsed() >= max_duration {
                println!("Reached time limit of {} seconds", duration_seconds);
                break;
            }

            // Check if we should stop
            {
                let running = self.running.lock().map_err(|e| {
                    PyRuntimeError::new_err(format!("Failed to lock running flag: {}", e))
                })?;
                if !*running {
                    break;
                }
            }

            // Execute tick for filtered nodes in priority order
            {
                let mut nodes = self
                    .nodes
                    .lock()
                    .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

                nodes.sort_by_key(|r| r.priority);

                for registered in nodes.iter_mut() {
                    // Skip nodes not in the filter list
                    if !node_names.contains(&registered.name) {
                        continue;
                    }

                    // Check rate control
                    let now = Instant::now();
                    let elapsed_secs = (now - registered.last_tick).as_secs_f64();
                    let period_secs = 1.0 / registered.rate_hz;

                    if elapsed_secs < period_secs {
                        continue;
                    }

                    registered.last_tick = now;

                    // Start tick timing
                    if let Ok(mut ctx) = registered.context.lock() {
                        ctx.start_tick();
                    }

                    // Get or create cached PyNodeInfo
                    let py_info = if let Some(ref cached) = registered.cached_info {
                        cached.clone_ref(py)
                    } else {
                        let new_info = Py::new(
                            py,
                            PyNodeInfo {
                                inner: registered.context.clone(),
                                scheduler_running: Some(self.running.clone()),
                            },
                        )?;
                        registered.cached_info = Some(new_info.clone_ref(py));
                        new_info
                    };

                    let result = registered
                        .node
                        .call_method1(py, "tick", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "tick"));

                    if let Err(e) = result {
                        // Check if this is a KeyboardInterrupt - if so, stop the scheduler immediately
                        if e.is_instance_of::<pyo3::exceptions::PyKeyboardInterrupt>(py) {
                            use colored::Colorize;
                            eprintln!(
                                "{}",
                                "\nKeyboard interrupt received, shutting down...".red()
                            );
                            if let Ok(mut r) = self.running.lock() {
                                *r = false;
                            }
                            break;
                        }
                        eprintln!("Error in node '{}' tick: {:?}", registered.name, e);
                    }

                    // Record tick completion
                    if let Ok(mut ctx) = registered.context.lock() {
                        ctx.record_tick();
                    }

                    // Write heartbeat for monitor
                    if let Ok(ctx) = registered.context.lock() {
                        Self::write_heartbeat(&registered.name, &ctx, registered.rate_hz);
                    }
                }
            }

            // Sleep for remainder of tick period
            let elapsed = tick_start.elapsed();
            if elapsed < tick_duration {
                thread::sleep(tick_duration - elapsed);
            }
        }

        // Shutdown filtered nodes
        {
            let nodes = self
                .nodes
                .lock()
                .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

            for registered in nodes.iter() {
                if node_names.contains(&registered.name) {
                    let py_info = Py::new(
                        py,
                        PyNodeInfo {
                            inner: registered.context.clone(),
                            scheduler_running: Some(self.running.clone()),
                        },
                    )?;

                    let result = registered
                        .node
                        .call_method1(py, "shutdown", (py_info,))
                        .or_else(|_| registered.node.call_method0(py, "shutdown"));

                    if let Err(e) = result {
                        eprintln!("Failed to shutdown node '{}': {:?}", registered.name, e);
                    }
                }
            }
        }

        Ok(())
    }

    /// Get list of added node names
    fn get_nodes(&self) -> PyResult<Vec<String>> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
        Ok(nodes.iter().map(|n| n.name.clone()).collect())
    }

    /// Get node information including priority and logging settings
    fn get_node_info(&self, name: String) -> PyResult<Option<(u32, bool)>> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;

        for registered in nodes.iter() {
            if registered.name == name {
                return Ok(Some((registered.priority, registered.logging_enabled)));
            }
        }
        Ok(None)
    }

    fn __repr__(&self) -> PyResult<String> {
        let nodes = self
            .nodes
            .lock()
            .map_err(|e| PyRuntimeError::new_err(format!("Failed to lock nodes: {}", e)))?;
        Ok(format!(
            "Scheduler(nodes={}, tick_rate={}Hz)",
            nodes.len(),
            self.tick_rate_hz
        ))
    }

    /// Pickle support: Get state for serialization
    fn __getstate__(&self, py: Python) -> PyResult<PyObject> {
        use pyo3::types::PyDict;

        let state = PyDict::new(py);
        state.set_item("tick_rate_hz", self.tick_rate_hz)?;

        // Note: Registered nodes cannot be serialized (contain PyObject references)
        // After unpickling, users must re-add nodes using scheduler.add()

        Ok(state.into())
    }

    /// Pickle support: Restore state from deserialization
    fn __setstate__(&mut self, state: &Bound<'_, pyo3::types::PyDict>) -> PyResult<()> {
        let tick_rate_hz: f64 = state
            .get_item("tick_rate_hz")?
            .ok_or_else(|| PyRuntimeError::new_err("Missing 'tick_rate_hz' in pickled state"))?
            .extract()?;

        // Recreate scheduler with empty nodes list
        Self::setup_heartbeat_directory();

        self.tick_rate_hz = tick_rate_hz;
        self.nodes = Arc::new(Mutex::new(Vec::new()));
        self.running = Arc::new(Mutex::new(false));

        Ok(())
    }
}

impl PyScheduler {
    /// Create heartbeat directory for monitor
    fn setup_heartbeat_directory() {
        let dir = shm_heartbeats_dir();
        let _ = fs::create_dir_all(&dir);
    }

    /// Write heartbeat for a node (for monitor)
    fn write_heartbeat(node_name: &str, context: &CoreNodeInfo, rate_hz: f64) {
        let heartbeat = NodeHeartbeat::from_metrics(context.state().clone(), context.metrics());

        // Override target_rate_hz with actual node rate
        let mut heartbeat = heartbeat;
        heartbeat.target_rate_hz = rate_hz as u32;

        let _ = heartbeat.write_to_file(node_name);
    }

    /// Clean up all heartbeat files
    fn cleanup_heartbeats() {
        let dir = shm_heartbeats_dir();
        if dir.exists() {
            // Only remove files, not the directory (other processes may be using it)
            if let Ok(entries) = fs::read_dir(&dir) {
                for entry in entries.flatten() {
                    let _ = fs::remove_file(entry.path());
                }
            }
        }
    }

    /// Clean up session directory (no-op with flat namespace)
    ///
    /// With the simplified flat namespace model, topics are shared globally.
    /// Use `horus clean --shm` to remove all shared memory.
    fn cleanup_session() {
        // No-op: flat namespace means no session-specific cleanup needed
    }

    /// Get path to registry file (cross-platform)
    fn get_registry_path() -> Result<PathBuf, std::io::Error> {
        let mut path = dirs::home_dir().unwrap_or_else(std::env::temp_dir);
        path.push(".horus_registry.json");
        Ok(path)
    }

    /// Write metadata to registry file for monitor to read
    fn update_registry(
        nodes: &[RegisteredNode],
        scheduler_name: &str,
        working_dir: &std::path::Path,
    ) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();

            // Collect node info
            let nodes_json: Vec<String> = nodes
                .iter()
                .map(|registered| {
                    let name = &registered.name;
                    let priority = registered.priority;

                    // Format publishers array
                    let pubs_json: Vec<String> = registered
                        .publishers
                        .iter()
                        .map(|t| format!("\"{}\"", t))
                        .collect();
                    let pubs_str = pubs_json.join(", ");

                    // Format subscribers array
                    let subs_json: Vec<String> = registered
                        .subscribers
                        .iter()
                        .map(|t| format!("\"{}\"", t))
                        .collect();
                    let subs_str = subs_json.join(", ");

                    format!(
                        "    {{\"name\": \"{}\", \"priority\": {}, \"publishers\": [{}], \"subscribers\": [{}]}}",
                        name, priority, pubs_str, subs_str
                    )
                })
                .collect();

            let registry_data = format!(
                "{{\n  \"pid\": {},\n  \"scheduler_name\": \"{}\",\n  \"working_dir\": \"{}\",\n  \"nodes\": [\n{}\n  ]\n}}",
                pid,
                scheduler_name,
                working_dir.to_string_lossy(),
                nodes_json.join(",\n")
            );

            let _ = fs::write(&registry_path, registry_data);
        }
    }

    /// Snapshot node state to registry (for crash forensics and persistence)
    /// Called every 5 seconds to avoid I/O overhead
    fn snapshot_state_to_registry(
        nodes: &[RegisteredNode],
        scheduler_name: &str,
        working_dir: &std::path::Path,
    ) {
        if let Ok(registry_path) = Self::get_registry_path() {
            let pid = std::process::id();
            let timestamp = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_secs();

            // Collect node info including state and health
            let nodes_json: Vec<String> = nodes
                .iter()
                .map(|registered| {
                    let name = &registered.name;
                    let priority = registered.priority;

                    // Get state and health from context
                    let (state_str, health_str, error_count, tick_count) =
                        if let Ok(ctx) = registered.context.lock() {
                            let heartbeat = NodeHeartbeat::from_metrics(
                                ctx.state().clone(),
                                ctx.metrics(),
                            );
                            (
                                ctx.state().to_string(),
                                heartbeat.health.as_str().to_string(),
                                ctx.metrics().errors_count,
                                ctx.metrics().total_ticks,
                            )
                        } else {
                            (
                                "Unknown".to_string(),
                                "Unknown".to_string(),
                                0,
                                0,
                            )
                        };

                    // Format publishers array
                    let pubs_json: Vec<String> = registered
                        .publishers
                        .iter()
                        .map(|t| format!("\"{}\"", t))
                        .collect();
                    let pubs_str = pubs_json.join(", ");

                    // Format subscribers array
                    let subs_json: Vec<String> = registered
                        .subscribers
                        .iter()
                        .map(|t| format!("\"{}\"", t))
                        .collect();
                    let subs_str = subs_json.join(", ");

                    format!(
                        "    {{\"name\": \"{}\", \"priority\": {}, \"state\": \"{}\", \"health\": \"{}\", \"error_count\": {}, \"tick_count\": {}, \"publishers\": [{}], \"subscribers\": [{}]}}",
                        name, priority, state_str, health_str, error_count, tick_count, pubs_str, subs_str
                    )
                })
                .collect();

            let registry_data = format!(
                "{{\n  \"pid\": {},\n  \"scheduler_name\": \"{}\",\n  \"working_dir\": \"{}\",\n  \"last_snapshot\": {},\n  \"nodes\": [\n{}\n  ]\n}}",
                pid,
                scheduler_name,
                working_dir.to_string_lossy(),
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

    /// Remove registry file when scheduler stops
    fn cleanup_registry() {
        if let Ok(registry_path) = Self::get_registry_path() {
            let _ = fs::remove_file(registry_path);
        }
    }
}
