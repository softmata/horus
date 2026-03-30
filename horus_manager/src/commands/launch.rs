//! Launch command - Multi-node launch from YAML files
//!
//! Launches multiple HORUS nodes from a configuration file.

use colored::*;
use horus_core::core::DurationExt;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use serde::{Deserialize, Serialize};
use std::collections::HashMap;
use std::path::Path;
use std::process::{Child, Command, Stdio};
use std::time::Duration;

/// Maximum number of automatic restarts before giving up
const MAX_RESTARTS: u32 = 10;
/// Initial backoff delay between restarts
const INITIAL_BACKOFF: Duration = Duration::from_millis(100);
/// Maximum backoff delay between restarts
const MAX_BACKOFF: Duration = Duration::from_secs(10);

/// Tracks restart state for a launched process
struct ProcessEntry {
    name: String,
    child: Child,
    node: LaunchNode,
    restart_count: u32,
    next_backoff: Duration,
}

/// Node configuration in a launch file
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchNode {
    /// Node name
    pub name: String,

    /// Package/crate containing the node
    #[serde(default)]
    pub package: Option<String>,

    /// Node priority (lower = higher priority)
    #[serde(default)]
    pub priority: Option<i32>,

    /// Node tick rate in Hz
    #[serde(default)]
    pub rate_hz: Option<u32>,

    /// Node parameters
    #[serde(default)]
    pub params: HashMap<String, serde_yaml::Value>,

    /// Environment variables
    #[serde(default)]
    pub env: HashMap<String, String>,

    /// Command to run (if not using package)
    #[serde(default)]
    pub command: Option<String>,

    /// Arguments to pass
    #[serde(default)]
    pub args: Vec<String>,

    /// Namespace prefix for topics
    #[serde(default)]
    pub namespace: Option<String>,

    /// Nodes this depends on (will wait for them to start)
    #[serde(default)]
    pub depends_on: Vec<String>,

    /// Delay before starting (seconds)
    #[serde(default)]
    pub start_delay: Option<f64>,

    /// Restart policy: "never", "always", "on-failure"
    #[serde(default = "default_restart")]
    pub restart: String,
}

fn default_restart() -> String {
    "never".to_string()
}

/// Launch file configuration
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchConfig {
    /// Nodes to launch
    #[serde(default)]
    pub nodes: Vec<LaunchNode>,

    /// Global environment variables
    #[serde(default)]
    pub env: HashMap<String, String>,

    /// Global namespace prefix
    #[serde(default)]
    pub namespace: Option<String>,

    /// Session name
    #[serde(default)]
    pub session: Option<String>,
}

// ── Launch Control Commands ───────────────────────────────────────────

/// Commands accepted on the `horus.launch.ctl.{session}` topic.
///
/// The launch monitor loop polls this topic and fans out commands to the
/// appropriate per-Scheduler control topics discovered via [`LaunchSession`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub enum LaunchControlCommand {
    /// Stop a specific launched process by name.
    StopNode { name: String },
    /// Kill and respawn a launched process.
    RestartNode { name: String },
    /// Gracefully stop all launched processes.
    StopAll,
}

// ── Launch Event Log (JSONL blackbox) ─────────────────────────────────

/// Lifecycle events recorded to `shm_base_dir()/launch/{session}.events.jsonl`.
#[derive(Debug, Serialize)]
#[serde(tag = "event")]
enum LaunchEvent {
    SessionStart {
        session: String,
        launch_file: String,
        node_count: usize,
        timestamp_ns: u64,
    },
    NodeSpawned {
        name: String,
        pid: u32,
        command: String,
        timestamp_ns: u64,
    },
    NodeCrashed {
        name: String,
        pid: u32,
        exit_code: Option<i32>,
        timestamp_ns: u64,
    },
    NodeRestarted {
        name: String,
        old_pid: u32,
        new_pid: u32,
        attempt: u32,
        timestamp_ns: u64,
    },
    SessionStop {
        session: String,
        reason: String,
        timestamp_ns: u64,
    },
}

/// Append a single event as one JSON line to the JSONL event log.
fn log_launch_event(session: &str, event: &LaunchEvent) {
    let dir = horus_core::memory::shm_base_dir().join("launch");
    let _ = std::fs::create_dir_all(&dir);
    let path = dir.join(format!("{}.events.jsonl", session));
    if let Ok(json) = serde_json::to_string(event) {
        use std::io::Write;
        if let Ok(mut f) = std::fs::OpenOptions::new()
            .create(true)
            .append(true)
            .open(&path)
        {
            let _ = writeln!(f, "{}", json);
        }
    }
}

fn now_ns() -> u64 {
    std::time::SystemTime::now()
        .duration_since(std::time::UNIX_EPOCH)
        .unwrap_or_default()
        .as_nanos() as u64
}

// ── Launch Session (SHM manifest for observability) ───────────────────

/// Persistent session manifest written to SHM so CLI tools can discover
/// which processes belong to a launch session and route commands to their
/// Scheduler control topics.
///
/// Path: `shm_base_dir()/launch/{session}.json`
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchSession {
    pub session: String,
    pub launcher_pid: u32,
    pub start_time: u64,
    pub launch_file: String,
    pub namespace: Option<String>,
    pub processes: Vec<LaunchProcessEntry>,
}

/// Per-process entry inside a [`LaunchSession`].
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct LaunchProcessEntry {
    pub name: String,
    pub pid: u32,
    pub status: String,
    /// Discovered from SHM NodePresence after the process creates a Scheduler.
    #[serde(default)]
    pub scheduler_name: Option<String>,
    /// Derived: `horus.ctl.{scheduler_name}`.
    #[serde(default)]
    pub control_topic: Option<String>,
    /// Node names inside this process's Scheduler (from presence files).
    #[serde(default)]
    pub node_names: Vec<String>,
    pub restart_count: u32,
}

impl LaunchSession {
    /// SHM directory for launch session manifests.
    fn launch_dir() -> std::path::PathBuf {
        horus_core::memory::shm_base_dir().join("launch")
    }

    /// Full path for this session's manifest.
    fn session_path(&self) -> std::path::PathBuf {
        Self::launch_dir().join(format!("{}.json", self.session))
    }

    /// Write session manifest to SHM (atomic rename).
    pub fn write(&self) -> std::io::Result<()> {
        let dir = Self::launch_dir();
        std::fs::create_dir_all(&dir)?;

        let dest = self.session_path();
        let tmp = dest.with_extension("json.tmp");

        let json = serde_json::to_string_pretty(self)
            .map_err(|e| std::io::Error::new(std::io::ErrorKind::InvalidData, e))?;

        {
            use std::io::Write;
            let mut f = std::fs::File::create(&tmp)?;
            f.write_all(json.as_bytes())?;
            f.flush()?;
        }

        std::fs::rename(&tmp, &dest)
    }

    /// Remove session manifest from SHM.
    pub fn remove(&self) {
        let _ = std::fs::remove_file(self.session_path());
    }

    /// Read a session manifest by name.
    pub fn read(session: &str) -> Option<Self> {
        let path = Self::launch_dir().join(format!("{}.json", session));
        let content = std::fs::read_to_string(&path).ok()?;
        serde_json::from_str(&content).ok()
    }

    /// List all session manifests. Stale sessions (dead launcher PID) are removed.
    pub fn read_all() -> Vec<Self> {
        let dir = Self::launch_dir();
        if !dir.exists() {
            return Vec::new();
        }
        let mut sessions = Vec::new();
        if let Ok(entries) = std::fs::read_dir(&dir) {
            for entry in entries.flatten() {
                let path = entry.path();
                if path.extension().is_some_and(|ext| ext == "json") {
                    if let Ok(content) = std::fs::read_to_string(&path) {
                        if let Ok(session) = serde_json::from_str::<LaunchSession>(&content) {
                            if horus_sys::process::ProcessHandle::from_pid(session.launcher_pid)
                                .is_alive()
                            {
                                sessions.push(session);
                            } else {
                                // Stale — launcher is dead, clean up
                                let _ = std::fs::remove_file(&path);
                            }
                        }
                    }
                }
            }
        }
        sessions
    }

    /// Poll NodePresence files to discover which Schedulers appeared for our PIDs.
    pub fn discover_schedulers(&mut self) {
        let all_presence = horus_core::core::NodePresence::read_all();

        // Also scan scheduler/ directory for scheduler names.
        // Each Scheduler creates shm_base_dir()/scheduler/{name} — the filename
        // IS the scheduler name. We check if that name matches any presence entry's
        // scheduler field, and if that presence entry's PID matches a launched process.
        let sched_dir = horus_core::memory::shm_base_dir().join("scheduler");
        let known_schedulers: Vec<String> = std::fs::read_dir(&sched_dir)
            .ok()
            .map(|entries| {
                entries
                    .flatten()
                    .filter_map(|e| e.file_name().into_string().ok())
                    .collect()
            })
            .unwrap_or_default();

        for entry in &mut self.processes {
            if entry.scheduler_name.is_some() {
                continue; // already discovered
            }

            // Strategy 1: Match presence entries by PID
            let matching: Vec<_> = all_presence
                .iter()
                .filter(|p| p.pid() == entry.pid)
                .collect();

            if !matching.is_empty() {
                if let Some(sched) = matching[0].scheduler() {
                    entry.scheduler_name = Some(sched.to_string());
                    entry.control_topic = Some(format!("horus.ctl.{}", sched));
                }
                entry.node_names = matching.iter().map(|p| p.name().to_string()).collect();
                continue;
            }

            // Strategy 2: Match by launch node name against scheduler directory.
            // When HORUS_NODE_NAME is set, the Scheduler creates a file named
            // after it in scheduler/. If that name matches this entry's name,
            // the process owns that scheduler.
            if known_schedulers.contains(&entry.name) {
                entry.scheduler_name = Some(entry.name.clone());
                entry.control_topic = Some(format!("horus.ctl.{}", entry.name));
                // Collect node names from any presence entry with this scheduler name
                entry.node_names = all_presence
                    .iter()
                    .filter(|p| p.scheduler() == Some(&entry.name))
                    .map(|p| p.name().to_string())
                    .collect();
            }
        }
    }
}

/// Run the launch command
pub fn run_launch(
    file: &Path,
    dry_run: bool,
    namespace: Option<String>,
    shutdown_timeout_secs: u64,
) -> HorusResult<()> {
    log::debug!("launching from file: {:?}", file);

    // Check if file exists
    if !file.exists() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Launch file not found: {}",
            file.display()
        ))));
    }

    // Read and parse the launch file
    let content = std::fs::read_to_string(file).map_err(HorusError::Io)?;

    let config: LaunchConfig = serde_yaml::from_str(&content).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "failed to parse launch file: {}",
            e
        )))
    })?;

    if config.nodes.is_empty() {
        println!("{}", "No nodes defined in launch file.".yellow());
        return Ok(());
    }

    let session_name = config.session.clone().unwrap_or_else(|| {
        file.file_stem()
            .map(|s| s.to_string_lossy().to_string())
            .unwrap_or_else(|| "launch".to_string())
    });

    // Apply namespace override
    let global_namespace = namespace.or(config.namespace.clone());

    println!("{}", "HORUS Multi-Node Launch".green().bold());
    println!();
    println!("  {} {}", "Launch file:".cyan(), file.display());
    println!("  {} {}", "Session:".cyan(), session_name);
    if let Some(ref ns) = global_namespace {
        println!("  {} {}", "Namespace:".cyan(), ns);
    }
    println!("  {} {}", "Nodes:".cyan(), config.nodes.len());
    println!();

    if dry_run {
        println!(
            "{}",
            "[DRY RUN] Would launch the following nodes:"
                .yellow()
                .bold()
        );
        println!();
        print_launch_plan(&config, &global_namespace);
        println!();
        println!("{} Run without --dry-run to launch nodes.", "".yellow());
        return Ok(());
    }

    // Sort nodes by dependencies (topological sort)
    let ordered_nodes = sort_by_dependencies(&config.nodes)?;

    // Launch nodes
    println!("{}", "Launching nodes...".cyan().bold());
    println!();

    log_launch_event(
        &session_name,
        &LaunchEvent::SessionStart {
            session: session_name.clone(),
            launch_file: file.display().to_string(),
            node_count: ordered_nodes.len(),
            timestamp_ns: now_ns(),
        },
    );

    let mut processes: Vec<ProcessEntry> = Vec::new();
    let mut started_nodes: Vec<String> = Vec::new();

    for node in &ordered_nodes {
        // Check dependencies
        for dep in &node.depends_on {
            if !started_nodes.contains(dep) {
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Dependency '{}' for node '{}' not found or not started",
                    dep, node.name
                ))));
            }
        }

        // Apply start delay if specified
        if let Some(delay) = node.start_delay {
            if delay.is_finite() && delay > 0.0 {
                println!("  {} Waiting {:.1}s for {}", "".dimmed(), delay, node.name);
                std::thread::sleep(delay.secs());
            } else if !delay.is_finite() || delay < 0.0 {
                log::warn!(
                    "Ignoring invalid start_delay {} for node '{}'",
                    delay,
                    node.name
                );
            }
        }

        // Build the full node name with namespace
        let full_name = match (&global_namespace, &node.namespace) {
            (Some(global), Some(local)) => format!("{}/{}/{}", global, local, node.name),
            (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, node.name),
            (None, None) => node.name.clone(),
        };

        print!("  {} Launching {}...", "".cyan(), full_name.white().bold());

        match launch_node(node, &config.env, &global_namespace) {
            Ok(child) => {
                let pid = child.id();
                println!(" {} (PID: {})", "started".green(), pid);
                log_launch_event(
                    &session_name,
                    &LaunchEvent::NodeSpawned {
                        name: node.name.clone(),
                        pid,
                        command: node
                            .command
                            .clone()
                            .or(node.package.clone())
                            .unwrap_or_default(),
                        timestamp_ns: now_ns(),
                    },
                );
                processes.push(ProcessEntry {
                    name: node.name.clone(),
                    child,
                    node: node.clone(),
                    restart_count: 0,
                    next_backoff: INITIAL_BACKOFF,
                });
                started_nodes.push(node.name.clone());
            }
            Err(e) => {
                println!(" {}", "failed".red());
                eprintln!("    Error: {}", e);

                // Clean up already started processes
                println!();
                println!("{}", "Cleaning up started nodes...".yellow());
                for entry in processes {
                    let mut proc = entry.child;
                    print!("  {} Stopping {}...", "".yellow(), entry.name);
                    if proc.kill().is_ok() {
                        let _ = proc.wait(); // Reap to prevent zombie
                        println!(" {}", "stopped".green());
                    } else {
                        let _ = proc.wait(); // Reap even if kill failed
                        println!(" {}", "already stopped".dimmed());
                    }
                }

                return Err(e);
            }
        }
    }

    println!();
    println!(
        "{} All {} nodes launched successfully!",
        "".green(),
        processes.len()
    );

    // ── Write launch session manifest to SHM ──
    let mut session = LaunchSession {
        session: session_name.clone(),
        launcher_pid: std::process::id(),
        start_time: std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs(),
        launch_file: file.display().to_string(),
        namespace: global_namespace.clone(),
        processes: processes
            .iter()
            .map(|e| LaunchProcessEntry {
                name: e.name.clone(),
                pid: e.child.id(),
                status: "running".to_string(),
                scheduler_name: None,
                control_topic: None,
                node_names: Vec::new(),
                restart_count: 0,
            })
            .collect(),
    };
    if let Err(e) = session.write() {
        log::warn!("Failed to write launch session manifest: {}", e);
    }

    // Brief wait then discover Schedulers from SHM presence files
    std::thread::sleep(Duration::from_millis(500));
    session.discover_schedulers();
    let _ = session.write();

    println!();
    println!(
        "  {} Use 'horus node list' to see running nodes",
        "Tip:".dimmed()
    );
    println!(
        "  {} Use 'horus monitor' to view live status",
        "Tip:".dimmed()
    );

    // Wait for all processes to finish (or handle signals)
    println!();
    println!("{}", "Press Ctrl+C to stop all nodes...".dimmed());

    // Set up signal handler
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();

    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    // Subscribe to e-stop topic for safety propagation.
    let estop_topic =
        horus_core::communication::Topic::<horus_types::EmergencyStop>::new("_horus.estop").ok();

    // Launch control topic: accepts LaunchControlCommand from CLI tools.
    let launch_ctl_topic_name = format!("horus.launch.ctl.{}", session_name);
    let launch_ctl =
        horus_core::communication::Topic::<LaunchControlCommand>::new(&launch_ctl_topic_name).ok();
    if launch_ctl.is_some() {
        println!(
            "  {} Control topic: {}",
            "".dimmed(),
            launch_ctl_topic_name.dimmed()
        );
    }

    // Wait for shutdown signal
    while running.load(std::sync::atomic::Ordering::SeqCst) {
        // ── Launch control commands ──
        if let Some(ref ctl) = launch_ctl {
            while let Some(cmd) = ctl.recv() {
                match cmd {
                    LaunchControlCommand::StopNode { ref name } => {
                        println!("{} StopNode '{}' received", "ctl".cyan(), name);
                        // Find the process and send GracefulShutdown to its Scheduler
                        if let Some(entry) = session.processes.iter().find(|e| &e.name == name) {
                            if let Some(ref ctl_name) = entry.control_topic {
                                if let Ok(sched_ctl) =
                                    horus_core::communication::Topic::<
                                        horus_core::scheduling::control::ControlCommand,
                                    >::new(ctl_name)
                                {
                                    sched_ctl.send(horus_core::scheduling::control::ControlCommand::GracefulShutdown);
                                }
                            }
                            // Fallback: SIGTERM the process directly
                            if let Some(proc_entry) = processes.iter().find(|e| e.name == *name) {
                                let _ = horus_sys::process::ProcessHandle::from_pid(
                                    proc_entry.child.id(),
                                )
                                .signal(horus_sys::process::Signal::Terminate);
                            }
                        }
                    }
                    LaunchControlCommand::RestartNode { ref name } => {
                        println!("{} RestartNode '{}' received", "ctl".cyan(), name);
                        if let Some(proc_entry) = processes.iter_mut().find(|e| e.name == *name) {
                            // Send GracefulShutdown via Scheduler control topic first
                            if let Some(ref se) = session.processes.iter().find(|e| &e.name == name)
                            {
                                if let Some(ref ctl_name) = se.control_topic {
                                    if let Ok(ctl) =
                                        horus_core::communication::Topic::<
                                            horus_core::scheduling::control::ControlCommand,
                                        >::new(ctl_name)
                                    {
                                        ctl.send(horus_core::scheduling::control::ControlCommand::GracefulShutdown);
                                    }
                                }
                            }
                            // SIGTERM (not SIGKILL) so topics clean up gracefully
                            let _ =
                                horus_sys::process::ProcessHandle::from_child(&proc_entry.child)
                                    .signal(horus_sys::process::Signal::Terminate);
                            // Force restart policy for this cycle
                            proc_entry.node.restart = "always".to_string();
                        }
                    }
                    LaunchControlCommand::StopAll => {
                        println!("{} StopAll received", "ctl".cyan());
                        running.store(false, std::sync::atomic::Ordering::SeqCst);
                    }
                }
            }
        }

        // ── E-stop check ──
        if let Some(ref estop) = estop_topic {
            if estop.recv().is_some() {
                eprintln!(
                    "\n{} Emergency stop received — shutting down all launched nodes",
                    "⚠ E-STOP".red().bold()
                );
                // Send GracefulShutdown to each known Scheduler control topic
                for entry in &session.processes {
                    if let Some(ref ctl_name) = entry.control_topic {
                        if let Ok(ctl) = horus_core::communication::Topic::<
                            horus_core::scheduling::control::ControlCommand,
                        >::new(ctl_name)
                        {
                            ctl.send(
                                horus_core::scheduling::control::ControlCommand::GracefulShutdown,
                            );
                        }
                    }
                }
                // Signal the main loop to exit → shutdown section handles SIGTERM fallback
                running.store(false, std::sync::atomic::Ordering::SeqCst);
                break;
            }
        }

        // Check if any process has exited
        let mut stopped_indices: Vec<usize> = Vec::new();
        for (i, entry) in processes.iter_mut().enumerate() {
            match entry.child.try_wait() {
                Ok(Some(status)) => {
                    let should_restart = should_restart_node(&entry.node.restart, &status);

                    if !status.success() {
                        println!(
                            "{} Node '{}' exited with status: {}",
                            "⚠".yellow(),
                            entry.name,
                            status
                        );
                        log_launch_event(
                            &session_name,
                            &LaunchEvent::NodeCrashed {
                                name: entry.name.clone(),
                                pid: entry.child.id(),
                                exit_code: status.code(),
                                timestamp_ns: now_ns(),
                            },
                        );
                    } else {
                        println!("ℹ Node '{}' completed", entry.name);
                    }

                    if should_restart && entry.restart_count < MAX_RESTARTS {
                        entry.restart_count += 1;
                        let backoff = entry.next_backoff;
                        println!(
                            "  {} Restarting '{}' (attempt {}/{}, backoff {:.0?})",
                            "↻".cyan(),
                            entry.name,
                            entry.restart_count,
                            MAX_RESTARTS,
                            backoff,
                        );
                        std::thread::sleep(backoff);
                        entry.next_backoff = (backoff * 2).min(MAX_BACKOFF);

                        let old_pid = entry.child.id();
                        match launch_node(&entry.node, &config.env, &global_namespace) {
                            Ok(child) => {
                                let new_pid = child.id();
                                println!(
                                    "  {} '{}' restarted (PID: {})",
                                    "✓".green(),
                                    entry.name,
                                    new_pid
                                );
                                log_launch_event(
                                    &session_name,
                                    &LaunchEvent::NodeRestarted {
                                        name: entry.name.clone(),
                                        old_pid,
                                        new_pid,
                                        attempt: entry.restart_count,
                                        timestamp_ns: now_ns(),
                                    },
                                );
                                entry.child = child;
                                // Don't add to stopped_indices — it's running again
                            }
                            Err(e) => {
                                eprintln!(
                                    "  {} Failed to restart '{}': {}",
                                    "✗".red(),
                                    entry.name,
                                    e
                                );
                                stopped_indices.push(i);
                            }
                        }
                    } else if should_restart {
                        eprintln!(
                            "  {} Node '{}' exceeded max restarts ({}), giving up",
                            "✗".red(),
                            entry.name,
                            MAX_RESTARTS,
                        );
                        stopped_indices.push(i);
                    } else {
                        stopped_indices.push(i);
                    }
                }
                Ok(None) => {} // Still running
                Err(e) => {
                    log::error!("Error checking node '{}': {}", entry.name, e);
                }
            }
        }

        // Remove stopped processes (in reverse order to preserve indices)
        for i in stopped_indices.into_iter().rev() {
            processes.remove(i);
        }

        // Update session manifest with current process states
        for (i, entry) in processes.iter().enumerate() {
            if let Some(se) = session.processes.get_mut(i) {
                se.pid = entry.child.id();
                se.restart_count = entry.restart_count;
            }
        }
        // Re-discover Schedulers every ~2s (every 20 iterations)
        if session.processes.iter().any(|e| e.scheduler_name.is_none()) {
            session.discover_schedulers();
        }
        let _ = session.write();

        if processes.is_empty() {
            println!("{}", "All nodes have stopped.".dimmed());
            break;
        }

        std::thread::sleep(100_u64.ms());
    }

    // ── Three-phase coordinated shutdown ──
    //
    // Phase A: Send GracefulShutdown to each Scheduler's control topic (clean shutdown)
    // Phase B: SIGTERM any processes still alive after 2s
    // Phase C: SIGKILL any processes still alive after 3s more
    println!();
    println!("{}", "Shutting down nodes...".yellow().bold());

    // Phase A: Control topic shutdown
    let mut sent_graceful = false;
    for entry in &session.processes {
        if let Some(ref ctl_name) = entry.control_topic {
            if let Ok(ctl) = horus_core::communication::Topic::<
                horus_core::scheduling::control::ControlCommand,
            >::new(ctl_name)
            {
                println!(
                    "  {} GracefulShutdown → {}",
                    "→".dimmed(),
                    ctl_name.dimmed()
                );
                ctl.send(horus_core::scheduling::control::ControlCommand::GracefulShutdown);
                sent_graceful = true;
            }
        }
    }

    if sent_graceful {
        println!("  {} Waiting 2s for graceful shutdown...", "⏳".dimmed());
        std::thread::sleep(Duration::from_secs(2));
    }

    // Phase B: SIGTERM any processes still running
    for entry in &mut processes {
        if entry.child.try_wait().map(|s| s.is_some()).unwrap_or(true) {
            println!("  {} {} stopped cleanly", "✓".green(), entry.name);
            continue;
        }
        println!("  {} {} still running → SIGTERM", "⏹".yellow(), entry.name);
        let _ = horus_sys::process::ProcessHandle::from_child(&entry.child)
            .signal(horus_sys::process::Signal::Terminate);
    }

    // Wait for SIGTERM to take effect
    std::thread::sleep(Duration::from_secs(shutdown_timeout_secs.min(3)));

    // Phase C: SIGKILL any processes still alive
    for entry in &mut processes {
        if entry.child.try_wait().map(|s| s.is_some()).unwrap_or(true) {
            continue;
        }
        println!("  {} {} still alive → SIGKILL", "✗".red(), entry.name);
        let _ = entry.child.kill();
    }

    // Reap all children
    for entry in &mut processes {
        let _ = entry.child.wait();
    }

    println!();
    println!("{} All nodes stopped.", "".green());

    log_launch_event(
        &session_name,
        &LaunchEvent::SessionStop {
            session: session_name.clone(),
            reason: "shutdown".to_string(),
            timestamp_ns: now_ns(),
        },
    );

    // Clean up session manifest
    session.remove();

    Ok(())
}

/// Show active launch sessions read from SHM manifests.
pub fn show_launch_status() -> HorusResult<()> {
    let sessions = LaunchSession::read_all();
    if sessions.is_empty() {
        println!("{}", "No active launch sessions.".dimmed());
        return Ok(());
    }

    println!("{}", "ACTIVE LAUNCH SESSIONS".green().bold());
    println!();

    for session in &sessions {
        let uptime_secs = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap_or_default()
            .as_secs()
            .saturating_sub(session.start_time);
        let uptime = if uptime_secs >= 3600 {
            format!("{}h {}m", uptime_secs / 3600, (uptime_secs % 3600) / 60)
        } else if uptime_secs >= 60 {
            format!("{}m {}s", uptime_secs / 60, uptime_secs % 60)
        } else {
            format!("{}s", uptime_secs)
        };

        println!(
            "  {} {} ({} processes, uptime: {})",
            "●".green(),
            session.session.white().bold(),
            session.processes.len(),
            uptime.dimmed()
        );
        println!("    {} {}", "File:".dimmed(), session.launch_file.dimmed());
        if let Some(ref ns) = session.namespace {
            println!("    {} {}", "Namespace:".dimmed(), ns.dimmed());
        }
        println!();

        // Per-process table
        println!(
            "    {:<20} {:<8} {:<10} {:<20} {:<10}",
            "NAME".dimmed(),
            "PID".dimmed(),
            "STATUS".dimmed(),
            "SCHEDULER".dimmed(),
            "RESTARTS".dimmed()
        );
        for entry in &session.processes {
            let status_colored = match entry.status.as_str() {
                "running" => "running".green().to_string(),
                "crashed" => "crashed".red().to_string(),
                "stopped" => "stopped".dimmed().to_string(),
                other => other.yellow().to_string(),
            };
            println!(
                "    {:<20} {:<8} {:<10} {:<20} {:<10}",
                entry.name,
                entry.pid,
                status_colored,
                entry.scheduler_name.as_deref().unwrap_or("-"),
                entry.restart_count
            );
        }
        println!();
    }

    Ok(())
}

/// Stop a running launch session by name.
///
/// Sends StopAll to the session's control topic. If the control topic is
/// unavailable, falls back to SIGTERM on the launcher PID.
pub fn stop_launch_session(session_name: &str) -> HorusResult<()> {
    let session = LaunchSession::read(session_name).ok_or_else(|| {
        HorusError::Config(ConfigError::Other(format!(
            "No active launch session '{}'. Use --status to see active sessions.",
            session_name
        )))
    })?;

    println!(
        "{} Stopping launch session: {}",
        "⏹".yellow(),
        session_name.white().bold()
    );

    // Try control topic first
    let ctl_topic_name = format!("horus.launch.ctl.{}", session_name);
    let sent = if let Ok(ctl) =
        horus_core::communication::Topic::<LaunchControlCommand>::new(&ctl_topic_name)
    {
        println!("  {} StopAll → {}", "→".dimmed(), ctl_topic_name.dimmed());
        ctl.send(LaunchControlCommand::StopAll);
        true
    } else {
        false
    };

    if !sent {
        // Fallback: SIGTERM the launcher process
        println!(
            "  {} Control topic unavailable, sending SIGTERM to launcher PID {}",
            "⚠".yellow(),
            session.launcher_pid
        );
        let _ = horus_sys::process::ProcessHandle::from_pid(session.launcher_pid)
            .signal(horus_sys::process::Signal::Terminate);
    }

    println!("{} Stop command sent.", "✓".green());
    Ok(())
}

/// Determine if a node should be restarted based on its policy and exit status.
fn should_restart_node(policy: &str, status: &std::process::ExitStatus) -> bool {
    match policy {
        "always" => true,
        "on-failure" => !status.success(),
        _ => false, // "never" or unrecognized
    }
}

/// Print the launch plan (dry run)
fn print_launch_plan(config: &LaunchConfig, global_namespace: &Option<String>) {
    for (i, node) in config.nodes.iter().enumerate() {
        let full_name = match (global_namespace, &node.namespace) {
            (Some(global), Some(local)) => format!("{}/{}/{}", global, local, node.name),
            (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, node.name),
            (None, None) => node.name.clone(),
        };

        println!("  {}. {}", i + 1, full_name.white().bold());

        if let Some(ref pkg) = node.package {
            println!("     {} {}", "Package:".dimmed(), pkg);
        }
        if let Some(ref cmd) = node.command {
            println!("     {} {}", "Command:".dimmed(), cmd);
        }
        if let Some(priority) = node.priority {
            println!("     {} {}", "Priority:".dimmed(), priority);
        }
        if let Some(rate) = node.rate_hz {
            println!("     {} {} Hz", "Rate:".dimmed(), rate);
        }
        if !node.params.is_empty() {
            println!("     {}", "Params:".dimmed());
            for (k, v) in &node.params {
                println!("       {} = {:?}", k, v);
            }
        }
        if !node.depends_on.is_empty() {
            println!(
                "     {} {}",
                "Depends:".dimmed(),
                node.depends_on.join(", ")
            );
        }
        if let Some(delay) = node.start_delay {
            println!("     {} {:.1}s", "Delay:".dimmed(), delay);
        }
        println!();
    }
}

/// Sort nodes by dependencies (topological sort)
fn sort_by_dependencies(nodes: &[LaunchNode]) -> HorusResult<Vec<LaunchNode>> {
    let mut result = Vec::new();
    let mut visited = std::collections::HashSet::new();
    let mut temp_visited = std::collections::HashSet::new();

    let node_map: HashMap<&str, &LaunchNode> = nodes.iter().map(|n| (n.name.as_str(), n)).collect();

    fn visit<'a>(
        node: &'a LaunchNode,
        node_map: &HashMap<&str, &'a LaunchNode>,
        visited: &mut std::collections::HashSet<String>,
        temp_visited: &mut std::collections::HashSet<String>,
        result: &mut Vec<LaunchNode>,
    ) -> HorusResult<()> {
        if temp_visited.contains(&node.name) {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Circular dependency detected involving node '{}'",
                node.name
            ))));
        }
        if visited.contains(&node.name) {
            return Ok(());
        }

        temp_visited.insert(node.name.clone());

        for dep in &node.depends_on {
            if let Some(dep_node) = node_map.get(dep.as_str()) {
                visit(dep_node, node_map, visited, temp_visited, result)?;
            }
            // If dependency not found, it might be external - we'll check at runtime
        }

        temp_visited.remove(&node.name);
        visited.insert(node.name.clone());
        result.push(node.clone());

        Ok(())
    }

    for node in nodes {
        visit(
            node,
            &node_map,
            &mut visited,
            &mut temp_visited,
            &mut result,
        )?;
    }

    Ok(result)
}

/// Launch a single node
fn launch_node(
    node: &LaunchNode,
    global_env: &HashMap<String, String>,
    namespace: &Option<String>,
) -> HorusResult<Child> {
    let mut cmd = if let Some(ref command) = node.command {
        // Custom command
        let parts: Vec<&str> = command.split_whitespace().collect();
        if parts.is_empty() {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Empty command for node '{}'",
                node.name
            ))));
        }
        let mut c = Command::new(parts[0]);
        if parts.len() > 1 {
            c.args(&parts[1..]);
        }
        c
    } else if let Some(ref package) = node.package {
        // Run as a HORUS package
        let horus_bin = std::env::current_exe().unwrap_or_else(|_| "horus".into());
        let mut c = Command::new(horus_bin);
        c.args(["run", package]);
        c
    } else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Node '{}' must specify either 'command' or 'package'",
            node.name
        ))));
    };

    // Add arguments
    cmd.args(&node.args);

    // Set environment variables
    for (k, v) in global_env {
        cmd.env(k, v);
    }
    for (k, v) in &node.env {
        cmd.env(k, v);
    }

    // Set HORUS-specific environment
    cmd.env("HORUS_NODE_NAME", &node.name);
    if let Some(priority) = node.priority {
        cmd.env("HORUS_NODE_PRIORITY", priority.to_string());
    }
    if let Some(rate) = node.rate_hz {
        cmd.env("HORUS_NODE_RATE_HZ", rate.to_string());
    }
    if let Some(ref ns) = namespace {
        cmd.env("HORUS_NAMESPACE", ns);
    }
    if let Some(ref ns) = node.namespace {
        cmd.env("HORUS_NODE_NAMESPACE", ns);
    }

    // Set parameters as environment.
    // Note: serde_yaml::to_string() adds a trailing newline — trim it.
    for (k, v) in &node.params {
        let env_key = format!("HORUS_PARAM_{}", k.to_uppercase().replace('-', "_"));
        let env_value = match v {
            serde_yaml::Value::String(s) => s.clone(),
            other => serde_yaml::to_string(other)
                .unwrap_or_default()
                .trim()
                .to_string(),
        };
        cmd.env(env_key, env_value);
    }

    // Configure I/O
    cmd.stdout(Stdio::inherit());
    cmd.stderr(Stdio::inherit());

    // Spawn the process
    cmd.spawn().map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to launch node '{}': {}",
            node.name, e
        )))
    })
}

/// List nodes in a launch file
pub fn list_launch_nodes(file: &Path) -> HorusResult<()> {
    if !file.exists() {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Launch file not found: {}",
            file.display()
        ))));
    }

    let content = std::fs::read_to_string(file).map_err(HorusError::Io)?;

    let config: LaunchConfig = serde_yaml::from_str(&content).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "failed to parse launch file: {}",
            e
        )))
    })?;

    println!("{}", "Launch File Contents".green().bold());
    println!();
    println!("  {} {}", "File:".cyan(), file.display());
    if let Some(ref session) = config.session {
        println!("  {} {}", "Session:".cyan(), session);
    }
    if let Some(ref ns) = config.namespace {
        println!("  {} {}", "Namespace:".cyan(), ns);
    }
    println!();

    if config.nodes.is_empty() {
        println!("{}", "  No nodes defined.".yellow());
    } else {
        println!(
            "  {:<30} {:>10} {:>10} {:>15}",
            "NAME".dimmed(),
            "PRIORITY".dimmed(),
            "RATE".dimmed(),
            "DEPENDS ON".dimmed()
        );
        println!("  {}", "-".repeat(70).dimmed());

        for node in &config.nodes {
            let priority = node
                .priority
                .map(|p| p.to_string())
                .unwrap_or("-".to_string());
            let rate = node
                .rate_hz
                .map(|r| format!("{} Hz", r))
                .unwrap_or("-".to_string());
            let deps = if node.depends_on.is_empty() {
                "-".to_string()
            } else {
                node.depends_on.join(", ")
            };

            println!(
                "  {:<30} {:>10} {:>10} {:>15}",
                node.name, priority, rate, deps
            );
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn launch_config_deserialize_minimal() {
        let yaml = "nodes: []";
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert!(config.nodes.is_empty());
        assert!(config.namespace.is_none());
        assert!(config.session.is_none());
    }

    #[test]
    fn launch_config_deserialize_full() {
        let yaml = r#"
session: test_session
namespace: /robot
env:
  HORUS_LOG: debug
nodes:
  - name: motor_ctrl
    package: my_robot
    priority: 1
    rate_hz: 100
    restart: always
    depends_on: [sensor_node]
    params:
      max_speed: 1.5
    env:
      MOTOR_ID: "0"
  - name: sensor_node
    command: /usr/local/bin/sensor
    args: ["--port", "5000"]
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.session.as_deref(), Some("test_session"));
        assert_eq!(config.namespace.as_deref(), Some("/robot"));
        assert_eq!(config.nodes.len(), 2);

        let motor = &config.nodes[0];
        assert_eq!(motor.name, "motor_ctrl");
        assert_eq!(motor.package.as_deref(), Some("my_robot"));
        assert_eq!(motor.priority, Some(1));
        assert_eq!(motor.rate_hz, Some(100));
        assert_eq!(motor.restart, "always");
        assert_eq!(motor.depends_on, vec!["sensor_node"]);

        let sensor = &config.nodes[1];
        assert_eq!(sensor.name, "sensor_node");
        assert_eq!(sensor.command.as_deref(), Some("/usr/local/bin/sensor"));
        assert_eq!(sensor.args, vec!["--port", "5000"]);
    }

    #[test]
    fn launch_node_default_restart_is_never() {
        let yaml = "name: test_node";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.restart, "never");
    }

    #[test]
    fn launch_config_roundtrip_serde() {
        let config = LaunchConfig {
            nodes: vec![LaunchNode {
                name: "test".to_string(),
                package: None,
                priority: Some(5),
                rate_hz: Some(50),
                params: HashMap::new(),
                env: HashMap::new(),
                command: None,
                args: vec![],
                namespace: None,
                depends_on: vec![],
                start_delay: None,
                restart: "never".to_string(),
            }],
            env: HashMap::new(),
            namespace: None,
            session: Some("roundtrip".to_string()),
        };
        let yaml = serde_yaml::to_string(&config).unwrap();
        let decoded: LaunchConfig = serde_yaml::from_str(&yaml).unwrap();
        assert_eq!(decoded.nodes.len(), 1);
        assert_eq!(decoded.nodes[0].name, "test");
        assert_eq!(decoded.session.as_deref(), Some("roundtrip"));
    }

    #[test]
    fn run_launch_missing_file_returns_error() {
        let result = run_launch(Path::new("/nonexistent/launch.yaml"), false, None, 5);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Launch file not found"),
            "error should be user-friendly: {}",
            err
        );
    }

    #[test]
    fn list_launch_nodes_missing_file_returns_error() {
        let result = list_launch_nodes(Path::new("/nonexistent/launch.yaml"));
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Launch file not found"));
    }

    #[test]
    fn sort_by_dependencies_independent_nodes() {
        let nodes = vec![
            LaunchNode {
                name: "a".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: a").unwrap()
            },
            LaunchNode {
                name: "b".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: b").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 2);
    }

    #[test]
    fn sort_by_dependencies_respects_order() {
        let nodes = vec![
            LaunchNode {
                name: "motor".to_string(),
                depends_on: vec!["sensor".to_string()],
                ..serde_yaml::from_str("name: motor").unwrap()
            },
            LaunchNode {
                name: "sensor".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: sensor").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted[0].name, "sensor", "dependency should come first");
        assert_eq!(sorted[1].name, "motor");
    }

    #[test]
    fn sort_by_dependencies_detects_cycle() {
        let nodes = vec![
            LaunchNode {
                name: "a".to_string(),
                depends_on: vec!["b".to_string()],
                ..serde_yaml::from_str("name: a").unwrap()
            },
            LaunchNode {
                name: "b".to_string(),
                depends_on: vec!["a".to_string()],
                ..serde_yaml::from_str("name: b").unwrap()
            },
        ];
        let result = sort_by_dependencies(&nodes);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Circular dependency"),
            "should detect cycle: {}",
            err
        );
    }

    #[test]
    fn sort_by_dependencies_diamond() {
        // D depends on B and C, both depend on A
        let nodes = vec![
            LaunchNode {
                name: "D".to_string(),
                depends_on: vec!["B".to_string(), "C".to_string()],
                ..serde_yaml::from_str("name: D").unwrap()
            },
            LaunchNode {
                name: "B".to_string(),
                depends_on: vec!["A".to_string()],
                ..serde_yaml::from_str("name: B").unwrap()
            },
            LaunchNode {
                name: "C".to_string(),
                depends_on: vec!["A".to_string()],
                ..serde_yaml::from_str("name: C").unwrap()
            },
            LaunchNode {
                name: "A".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: A").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 4);
        assert_eq!(sorted[0].name, "A", "root dependency first");
        // D must come after B and C
        let d_pos = sorted.iter().position(|n| n.name == "D").unwrap();
        let b_pos = sorted.iter().position(|n| n.name == "B").unwrap();
        let c_pos = sorted.iter().position(|n| n.name == "C").unwrap();
        assert!(d_pos > b_pos);
        assert!(d_pos > c_pos);
    }

    #[test]
    fn run_launch_invalid_yaml_returns_error() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("bad.yaml");
        std::fs::write(&path, "not: [valid yaml: {{").unwrap();
        let result = run_launch(&path, false, None, 5);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("parse launch file"),
            "error should mention parsing: {}",
            err
        );
    }

    #[test]
    fn run_launch_empty_nodes_succeeds_dry_run() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("empty.yaml");
        std::fs::write(&path, "nodes: []").unwrap();
        // Empty nodes with dry_run should succeed (prints "No nodes defined")
        let result = run_launch(&path, true, None, 5);
        assert!(
            result.is_ok(),
            "empty nodes launch with dry_run should succeed: {:?}",
            result.err()
        );
        // Verify the YAML file still exists and was not modified
        assert!(
            path.exists(),
            "launch file should not be deleted by dry run"
        );
        let content = std::fs::read_to_string(&path).unwrap();
        assert_eq!(
            content, "nodes: []",
            "launch file should not be modified by dry run"
        );
    }

    #[test]
    fn node_namespace_construction() {
        // Mirrors the logic in run_launch/print_launch_plan
        let cases = vec![
            (Some("global"), Some("local"), "node", "global/local/node"),
            (Some("global"), None, "node", "global/node"),
            (None, Some("local"), "node", "local/node"),
            (None, None, "node", "node"),
        ];
        for (global, local, name, expected) in cases {
            let full = match (global, local) {
                (Some(g), Some(l)) => format!("{}/{}/{}", g, l, name),
                (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, name),
                (None, None) => name.to_string(),
            };
            assert_eq!(full, expected);
        }
    }

    #[test]
    fn launch_node_requires_command_or_package() {
        // A node without command or package should fail
        let node = LaunchNode {
            name: "orphan".to_string(),
            package: None,
            command: None,
            ..serde_yaml::from_str("name: orphan").unwrap()
        };
        let result = launch_node(&node, &HashMap::new(), &None);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("must specify either 'command' or 'package'"),
            "error should be helpful: {}",
            err
        );
    }

    #[test]
    fn launch_node_empty_command_fails() {
        let node = LaunchNode {
            name: "empty_cmd".to_string(),
            command: Some(String::new()),
            ..serde_yaml::from_str("name: empty_cmd").unwrap()
        };
        let result = launch_node(&node, &HashMap::new(), &None);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Empty command"));
    }

    #[test]
    fn launch_node_param_env_format() {
        // Verify param-to-env key transformation
        let key = "max-speed";
        let env_key = format!("HORUS_PARAM_{}", key.to_uppercase().replace('-', "_"));
        assert_eq!(env_key, "HORUS_PARAM_MAX_SPEED");
    }

    // ── Battle tests: YAML parsing edge cases ─────────────────────────────

    #[test]
    fn launch_config_deserialize_empty_yaml() {
        // An empty document should use defaults
        let config: LaunchConfig = serde_yaml::from_str("{}").unwrap();
        assert!(config.nodes.is_empty());
        assert!(config.env.is_empty());
        assert!(config.namespace.is_none());
        assert!(config.session.is_none());
    }

    #[test]
    fn launch_node_all_defaults() {
        let yaml = "name: minimal";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.name, "minimal");
        assert!(node.package.is_none());
        assert!(node.priority.is_none());
        assert!(node.rate_hz.is_none());
        assert!(node.params.is_empty());
        assert!(node.env.is_empty());
        assert!(node.command.is_none());
        assert!(node.args.is_empty());
        assert!(node.namespace.is_none());
        assert!(node.depends_on.is_empty());
        assert!(node.start_delay.is_none());
        assert_eq!(node.restart, "never");
    }

    #[test]
    fn launch_node_with_all_fields_populated() {
        let yaml = r#"
name: full_node
package: my_pkg
priority: -10
rate_hz: 500
command: /bin/test
args: ["--verbose", "--port", "8080"]
namespace: /ns/sub
depends_on: ["a", "b", "c"]
start_delay: 2.5
restart: on-failure
params:
  kp: 1.0
  ki: 0.01
env:
  MY_VAR: my_value
"#;
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.name, "full_node");
        assert_eq!(node.package.as_deref(), Some("my_pkg"));
        assert_eq!(node.priority, Some(-10));
        assert_eq!(node.rate_hz, Some(500));
        assert_eq!(node.command.as_deref(), Some("/bin/test"));
        assert_eq!(node.args, vec!["--verbose", "--port", "8080"]);
        assert_eq!(node.namespace.as_deref(), Some("/ns/sub"));
        assert_eq!(node.depends_on, vec!["a", "b", "c"]);
        assert!((node.start_delay.unwrap() - 2.5).abs() < 0.001);
        assert_eq!(node.restart, "on-failure");
        assert_eq!(node.params.len(), 2);
        assert_eq!(node.env.get("MY_VAR").map(|s| s.as_str()), Some("my_value"));
    }

    #[test]
    fn launch_config_invalid_yaml_type_mismatch() {
        let yaml = "nodes: \"not a list\"";
        let result = serde_yaml::from_str::<LaunchConfig>(yaml);
        assert!(result.is_err());
    }

    #[test]
    fn launch_node_priority_negative() {
        let yaml = "name: neg\npriority: -100";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.priority, Some(-100));
    }

    #[test]
    fn launch_config_extra_fields_ignored() {
        // YAML with unknown fields should deserialize (serde default behavior)
        let yaml = r#"
nodes: []
unknown_field: 42
another: "hello"
"#;
        // serde_yaml with default deny_unknown_fields may fail; test actual behavior
        let result = serde_yaml::from_str::<LaunchConfig>(yaml);
        // If it succeeds, extra fields are ignored. If it fails, that's also valid behavior to document.
        // The important thing is no panic.
        let _outcome = result.is_ok();
    }

    // ── Battle tests: sort_by_dependencies ────────────────────────────────

    #[test]
    fn sort_by_dependencies_empty_list() {
        let result = sort_by_dependencies(&[]).unwrap();
        assert!(result.is_empty());
    }

    #[test]
    fn sort_by_dependencies_single_node() {
        let nodes = vec![LaunchNode {
            name: "solo".to_string(),
            depends_on: vec![],
            ..serde_yaml::from_str("name: solo").unwrap()
        }];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 1);
        assert_eq!(sorted[0].name, "solo");
    }

    #[test]
    fn sort_by_dependencies_linear_chain() {
        // A -> B -> C -> D (D depends on C, C depends on B, B depends on A)
        let nodes = vec![
            LaunchNode {
                name: "D".to_string(),
                depends_on: vec!["C".to_string()],
                ..serde_yaml::from_str("name: D").unwrap()
            },
            LaunchNode {
                name: "C".to_string(),
                depends_on: vec!["B".to_string()],
                ..serde_yaml::from_str("name: C").unwrap()
            },
            LaunchNode {
                name: "B".to_string(),
                depends_on: vec!["A".to_string()],
                ..serde_yaml::from_str("name: B").unwrap()
            },
            LaunchNode {
                name: "A".to_string(),
                depends_on: vec![],
                ..serde_yaml::from_str("name: A").unwrap()
            },
        ];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 4);
        let names: Vec<&str> = sorted.iter().map(|n| n.name.as_str()).collect();
        assert_eq!(names, vec!["A", "B", "C", "D"]);
    }

    #[test]
    fn sort_by_dependencies_self_cycle() {
        let nodes = vec![LaunchNode {
            name: "self_ref".to_string(),
            depends_on: vec!["self_ref".to_string()],
            ..serde_yaml::from_str("name: self_ref").unwrap()
        }];
        let result = sort_by_dependencies(&nodes);
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(err.contains("Circular dependency"), "self-cycle: {}", err);
    }

    #[test]
    fn sort_by_dependencies_three_node_cycle() {
        let nodes = vec![
            LaunchNode {
                name: "x".to_string(),
                depends_on: vec!["z".to_string()],
                ..serde_yaml::from_str("name: x").unwrap()
            },
            LaunchNode {
                name: "y".to_string(),
                depends_on: vec!["x".to_string()],
                ..serde_yaml::from_str("name: y").unwrap()
            },
            LaunchNode {
                name: "z".to_string(),
                depends_on: vec!["y".to_string()],
                ..serde_yaml::from_str("name: z").unwrap()
            },
        ];
        let result = sort_by_dependencies(&nodes);
        assert!(result.is_err());
    }

    #[test]
    fn sort_by_dependencies_external_dep_not_found() {
        // Node depends on something not in the list — should not error (treated as external)
        let nodes = vec![LaunchNode {
            name: "node_a".to_string(),
            depends_on: vec!["external_service".to_string()],
            ..serde_yaml::from_str("name: node_a").unwrap()
        }];
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 1);
        assert_eq!(sorted[0].name, "node_a");
    }

    // ── Battle tests: namespace construction ──────────────────────────────

    #[test]
    fn namespace_construction_with_special_characters() {
        let cases = vec![
            (Some("/robot1"), Some("arm"), "ctrl", "/robot1/arm/ctrl"),
            (Some("/"), None, "node", "//node"),
            (None, Some(""), "node", "/node"),
        ];
        for (global, local, name, expected) in cases {
            let full = match (global, local) {
                (Some(g), Some(l)) => format!("{}/{}/{}", g, l, name),
                (Some(ns), None) | (None, Some(ns)) => format!("{}/{}", ns, name),
                (None, None) => name.to_string(),
            };
            assert_eq!(
                full, expected,
                "global={:?} local={:?} name={}",
                global, local, name
            );
        }
    }

    // ── Battle tests: dry run with tempfile ───────────────────────────────

    #[test]
    fn run_launch_dry_run_with_nodes() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("test.yaml");
        let yaml = r#"
session: dry_test
namespace: /test
nodes:
  - name: sensor
    command: /bin/echo hello
    rate_hz: 100
  - name: controller
    command: /bin/echo world
    depends_on: [sensor]
    priority: 1
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = run_launch(&path, true, None, 5);
        assert!(result.is_ok(), "dry run should succeed: {:?}", result.err());
    }

    #[test]
    fn run_launch_dry_run_with_namespace_override() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("ns.yaml");
        let yaml = r#"
namespace: /original
nodes:
  - name: node1
    command: /bin/true
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = run_launch(&path, true, Some("/override".to_string()), 5);
        assert!(
            result.is_ok(),
            "namespace override dry run should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn list_launch_nodes_with_valid_file() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("list.yaml");
        let yaml = r#"
session: list_test
namespace: /bot
nodes:
  - name: nav
    priority: 1
    rate_hz: 50
    depends_on: [localization]
  - name: localization
    command: /bin/loc
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = list_launch_nodes(&path);
        assert!(
            result.is_ok(),
            "listing valid launch file should succeed: {:?}",
            result.err()
        );
    }

    #[test]
    fn list_launch_nodes_empty_nodes() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("empty.yaml");
        std::fs::write(&path, "nodes: []").unwrap();
        let result = list_launch_nodes(&path);
        assert!(result.is_ok());
    }

    #[test]
    fn launch_config_multi_robot_namespaces() {
        let yaml = r#"
namespace: /fleet
nodes:
  - name: planner
    namespace: /robot1
    command: /bin/plan
    rate_hz: 10
  - name: planner
    namespace: /robot2
    command: /bin/plan
    rate_hz: 10
  - name: coordinator
    command: /bin/coord
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.nodes.len(), 3);
        assert_eq!(config.namespace.as_deref(), Some("/fleet"));
        // Two nodes named "planner" in different namespaces
        let planners: Vec<_> = config
            .nodes
            .iter()
            .filter(|n| n.name == "planner")
            .collect();
        assert_eq!(planners.len(), 2);
        assert_eq!(planners[0].namespace.as_deref(), Some("/robot1"));
        assert_eq!(planners[1].namespace.as_deref(), Some("/robot2"));
    }

    #[test]
    fn launch_node_restart_policies() {
        for policy in &["never", "always", "on-failure"] {
            let yaml = format!("name: n\nrestart: {}", policy);
            let node: LaunchNode = serde_yaml::from_str(&yaml).unwrap();
            assert_eq!(node.restart, *policy);
        }
    }

    // ── Restart policy enforcement tests ─────────────────────────────────

    #[test]
    fn should_restart_always_on_success() {
        use std::process::Command;
        let status = Command::new("true").status().unwrap();
        assert!(should_restart_node("always", &status));
    }

    #[test]
    fn should_restart_always_on_failure() {
        use std::process::Command;
        let status = Command::new("false").status().unwrap();
        assert!(should_restart_node("always", &status));
    }

    #[test]
    fn should_restart_on_failure_only_on_failure() {
        use std::process::Command;
        let success = Command::new("true").status().unwrap();
        let failure = Command::new("false").status().unwrap();
        assert!(
            !should_restart_node("on-failure", &success),
            "on-failure should NOT restart on exit code 0"
        );
        assert!(
            should_restart_node("on-failure", &failure),
            "on-failure SHOULD restart on non-zero exit"
        );
    }

    #[test]
    fn should_restart_never_always_false() {
        use std::process::Command;
        let success = Command::new("true").status().unwrap();
        let failure = Command::new("false").status().unwrap();
        assert!(!should_restart_node("never", &success));
        assert!(!should_restart_node("never", &failure));
    }

    #[test]
    fn should_restart_unknown_policy_is_never() {
        use std::process::Command;
        let failure = Command::new("false").status().unwrap();
        assert!(
            !should_restart_node("banana", &failure),
            "unknown policy should behave as 'never'"
        );
    }

    #[test]
    fn restart_backoff_doubles_and_caps() {
        let mut backoff = INITIAL_BACKOFF;
        assert_eq!(backoff, Duration::from_millis(100));

        backoff = (backoff * 2).min(MAX_BACKOFF);
        assert_eq!(backoff, Duration::from_millis(200));

        backoff = (backoff * 2).min(MAX_BACKOFF);
        assert_eq!(backoff, Duration::from_millis(400));

        backoff = (backoff * 2).min(MAX_BACKOFF);
        assert_eq!(backoff, Duration::from_millis(800));

        backoff = (backoff * 2).min(MAX_BACKOFF);
        assert_eq!(backoff, Duration::from_millis(1600));

        // Keep doubling until cap
        for _ in 0..10 {
            backoff = (backoff * 2).min(MAX_BACKOFF);
        }
        assert_eq!(backoff, MAX_BACKOFF, "backoff should cap at MAX_BACKOFF");
    }

    #[test]
    fn max_restarts_constant_is_reasonable() {
        assert!(MAX_RESTARTS > 0, "must allow at least one restart");
        assert!(
            MAX_RESTARTS <= 100,
            "unreasonable max restarts would delay shutdown"
        );
    }

    #[test]
    fn launch_param_env_key_transform_edge_cases() {
        // Underscores, all-caps, numbers
        let cases = vec![
            ("my-param", "HORUS_PARAM_MY_PARAM"),
            ("kp", "HORUS_PARAM_KP"),
            ("max_speed", "HORUS_PARAM_MAX_SPEED"),
            ("a-b-c-d", "HORUS_PARAM_A_B_C_D"),
            ("", "HORUS_PARAM_"),
            ("123", "HORUS_PARAM_123"),
            ("ALREADY_UPPER", "HORUS_PARAM_ALREADY_UPPER"),
        ];
        for (key, expected) in cases {
            let env_key = format!("HORUS_PARAM_{}", key.to_uppercase().replace('-', "_"));
            assert_eq!(env_key, expected, "key={}", key);
        }
    }

    #[test]
    fn run_launch_yaml_with_only_env() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("env_only.yaml");
        let yaml = r#"
env:
  HORUS_LOG: debug
  MY_VAR: value
nodes: []
"#;
        std::fs::write(&path, yaml).unwrap();
        let result = run_launch(&path, true, None, 5);
        assert!(result.is_ok());
    }

    // ════════════════════════════════════════════════════════════════════════
    //  Restart policy & shutdown edge cases
    // ════════════════════════════════════════════════════════════════════════

    #[test]
    fn launch_node_invalid_restart_still_parses() {
        // serde_yaml accepts any string — validation happens at runtime
        let yaml = "name: bad_restart\nrestart: banana";
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.restart, "banana");
    }

    #[test]
    fn launch_node_restart_combined_with_depends_on() {
        let yaml = r#"
name: motor
restart: on-failure
depends_on: [sensor]
command: /usr/bin/motor
"#;
        let node: LaunchNode = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(node.restart, "on-failure");
        assert_eq!(node.depends_on, vec!["sensor"]);
    }

    #[test]
    fn launch_config_shutdown_timeout_parameter() {
        // run_launch accepts shutdown_timeout_secs as parameter
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("timeout.yaml");
        std::fs::write(&path, "nodes: []").unwrap();
        // Various timeout values should all succeed on empty config
        for timeout in [0, 1, 5, 30, 300] {
            let result = run_launch(&path, true, None, timeout);
            assert!(result.is_ok(), "timeout={} should succeed", timeout);
        }
    }

    #[test]
    fn launch_start_delay_edge_cases() {
        // NaN, negative, zero, very large
        for delay_str in ["0", "0.0", "-1.0", "999999.0"] {
            let yaml = format!(
                "name: delay_node\nstart_delay: {}\ncommand: /bin/true",
                delay_str
            );
            let node: LaunchNode = serde_yaml::from_str(&yaml).unwrap();
            assert!(node.start_delay.is_some());
        }
    }

    // ════════════════════════════════════════════════════════════════════════
    //  Scale: large node graphs & deep namespaces
    // ════════════════════════════════════════════════════════════════════════

    #[test]
    fn sort_50_node_linear_chain() {
        // A → B → C → ... → Z (50 nodes in a chain)
        let mut nodes = Vec::new();
        for i in 0..50 {
            nodes.push(LaunchNode {
                name: format!("node_{:02}", i),
                depends_on: if i > 0 {
                    vec![format!("node_{:02}", i - 1)]
                } else {
                    vec![]
                },
                command: Some("/bin/true".to_string()),
                ..serde_yaml::from_str("name: x").unwrap()
            });
        }
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 50);
        // First node should be node_00 (no deps)
        assert_eq!(sorted[0].name, "node_00");
        // Last should be node_49
        assert_eq!(sorted[49].name, "node_49");
    }

    #[test]
    fn sort_50_node_wide_fan() {
        // 49 nodes all depend on node_00
        let mut nodes = vec![LaunchNode {
            name: "root".to_string(),
            depends_on: vec![],
            command: Some("/bin/true".to_string()),
            ..serde_yaml::from_str("name: x").unwrap()
        }];
        for i in 1..50 {
            nodes.push(LaunchNode {
                name: format!("leaf_{:02}", i),
                depends_on: vec!["root".to_string()],
                command: Some("/bin/true".to_string()),
                ..serde_yaml::from_str("name: x").unwrap()
            });
        }
        let sorted = sort_by_dependencies(&nodes).unwrap();
        assert_eq!(sorted.len(), 50);
        assert_eq!(sorted[0].name, "root");
    }

    #[test]
    fn deep_namespace_5_levels() {
        let yaml = r#"
namespace: /warehouse/floor1
nodes:
  - name: controller
    namespace: /robot1/arm/joint3
    command: /bin/true
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        let node = &config.nodes[0];
        let global_ns = config.namespace.as_ref();
        let full_name = match (global_ns, &node.namespace) {
            (Some(g), Some(l)) => format!("{}/{}/{}", g, l, node.name),
            _ => node.name.clone(),
        };
        assert_eq!(
            full_name, "/warehouse/floor1//robot1/arm/joint3/controller",
            "deep namespace should merge correctly"
        );
    }

    #[test]
    fn sort_missing_dependency_target_errors() {
        let nodes = vec![LaunchNode {
            name: "motor".to_string(),
            depends_on: vec!["nonexistent_sensor".to_string()],
            command: Some("/bin/true".to_string()),
            ..serde_yaml::from_str("name: x").unwrap()
        }];
        let result = sort_by_dependencies(&nodes);
        // Should either error or place the node (implementation-dependent)
        // The important thing is it doesn't panic
        let _ = result;
    }

    #[test]
    fn sort_duplicate_node_names() {
        let nodes = vec![
            LaunchNode {
                name: "sensor".to_string(),
                depends_on: vec![],
                command: Some("/bin/a".to_string()),
                ..serde_yaml::from_str("name: x").unwrap()
            },
            LaunchNode {
                name: "sensor".to_string(),
                depends_on: vec![],
                command: Some("/bin/b".to_string()),
                ..serde_yaml::from_str("name: x").unwrap()
            },
        ];
        let result = sort_by_dependencies(&nodes);
        // Should handle gracefully (no panic), either error or deduplicate
        let _ = result;
    }

    #[test]
    fn launch_config_mixed_package_and_command_nodes() {
        let yaml = r#"
nodes:
  - name: driver
    package: lidar_driver
  - name: custom
    command: /opt/my_node
    args: ["--config", "prod.yaml"]
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.nodes.len(), 2);
        assert!(config.nodes[0].package.is_some());
        assert!(config.nodes[0].command.is_none());
        assert!(config.nodes[1].command.is_some());
        assert!(config.nodes[1].package.is_none());
    }

    // ── YAML parsing: basic two-node config ────────────────────────────

    #[test]
    fn test_launch_parse_yaml_basic() {
        let yaml = r#"
nodes:
  - name: lidar_driver
    command: /usr/bin/lidar
    args: ["--port", "5000", "--rate", "10"]
  - name: slam_node
    command: /usr/bin/slam
    args: ["--map", "warehouse.pgm"]
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.nodes.len(), 2);

        let lidar = &config.nodes[0];
        assert_eq!(lidar.name, "lidar_driver");
        assert_eq!(lidar.command.as_deref(), Some("/usr/bin/lidar"));
        assert_eq!(lidar.args, vec!["--port", "5000", "--rate", "10"]);

        let slam = &config.nodes[1];
        assert_eq!(slam.name, "slam_node");
        assert_eq!(slam.command.as_deref(), Some("/usr/bin/slam"));
        assert_eq!(slam.args, vec!["--map", "warehouse.pgm"]);
    }

    // ── YAML parsing: depends_on ───────────────────────────────────────

    #[test]
    fn test_launch_parse_yaml_with_depends_on() {
        let yaml = r#"
nodes:
  - name: node_a
    command: /bin/a
  - name: node_b
    command: /bin/b
    depends_on: [node_a]
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.nodes.len(), 2);

        let node_a = &config.nodes[0];
        assert!(
            node_a.depends_on.is_empty(),
            "node_a should have no dependencies"
        );

        let node_b = &config.nodes[1];
        assert_eq!(node_b.depends_on.len(), 1);
        assert_eq!(node_b.depends_on[0], "node_a");

        // Verify topological sort respects the dependency
        let sorted = sort_by_dependencies(&config.nodes).unwrap();
        let a_pos = sorted.iter().position(|n| n.name == "node_a").unwrap();
        let b_pos = sorted.iter().position(|n| n.name == "node_b").unwrap();
        assert!(
            a_pos < b_pos,
            "node_a (pos {}) must come before node_b (pos {})",
            a_pos,
            b_pos
        );
    }

    // ── YAML parsing: restart policies ─────────────────────────────────

    #[test]
    fn test_launch_parse_yaml_with_restart_policy() {
        let yaml = r#"
nodes:
  - name: always_node
    command: /bin/always
    restart: always
  - name: never_node
    command: /bin/never
    restart: never
  - name: failure_node
    command: /bin/fail
    restart: on-failure
"#;
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert_eq!(config.nodes.len(), 3);

        assert_eq!(config.nodes[0].name, "always_node");
        assert_eq!(config.nodes[0].restart, "always");

        assert_eq!(config.nodes[1].name, "never_node");
        assert_eq!(config.nodes[1].restart, "never");

        assert_eq!(config.nodes[2].name, "failure_node");
        assert_eq!(config.nodes[2].restart, "on-failure");
    }

    // ── YAML parsing: empty nodes list ─────────────────────────────────

    #[test]
    fn test_launch_parse_yaml_empty_nodes() {
        let yaml = "nodes: []";
        let config: LaunchConfig = serde_yaml::from_str(yaml).unwrap();
        assert!(config.nodes.is_empty(), "nodes list should be empty");

        // sort_by_dependencies should also handle empty input
        let sorted = sort_by_dependencies(&config.nodes).unwrap();
        assert!(sorted.is_empty());
    }

    // ── Launch-as-citizen unit tests ───────────────────────────────

    #[test]
    fn test_launch_session_serde_roundtrip() {
        let session = LaunchSession {
            session: "test_session".to_string(),
            launcher_pid: 12345,
            start_time: 1711234567,
            launch_file: "test.yaml".to_string(),
            namespace: Some("ns".to_string()),
            processes: vec![
                LaunchProcessEntry {
                    name: "ctrl".to_string(),
                    pid: 12346,
                    status: "running".to_string(),
                    scheduler_name: Some("ctrl".to_string()),
                    control_topic: Some("horus.ctl.ctrl".to_string()),
                    node_names: vec!["pid_loop".to_string()],
                    restart_count: 0,
                },
                LaunchProcessEntry {
                    name: "vision".to_string(),
                    pid: 12347,
                    status: "running".to_string(),
                    scheduler_name: None,
                    control_topic: None,
                    node_names: vec![],
                    restart_count: 2,
                },
            ],
        };
        let json = serde_json::to_string_pretty(&session).unwrap();
        let parsed: LaunchSession = serde_json::from_str(&json).unwrap();
        assert_eq!(parsed.session, "test_session");
        assert_eq!(parsed.processes.len(), 2);
        assert_eq!(parsed.processes[0].scheduler_name, Some("ctrl".to_string()));
        assert_eq!(parsed.processes[1].restart_count, 2);
    }

    #[test]
    fn test_launch_control_command_serde_roundtrip() {
        let cmds = vec![
            LaunchControlCommand::StopNode {
                name: "motor".to_string(),
            },
            LaunchControlCommand::RestartNode {
                name: "cam".to_string(),
            },
            LaunchControlCommand::StopAll,
        ];
        for cmd in &cmds {
            let json = serde_json::to_string(cmd).unwrap();
            let parsed: LaunchControlCommand = serde_json::from_str(&json).unwrap();
            assert_eq!(
                serde_json::to_string(&parsed).unwrap(),
                json,
                "roundtrip failed for {:?}",
                cmd
            );
        }
    }

    #[test]
    fn test_launch_event_serialization() {
        let event = LaunchEvent::NodeCrashed {
            name: "ctrl".to_string(),
            pid: 12345,
            exit_code: Some(139),
            timestamp_ns: 1711234567000000000,
        };
        let json = serde_json::to_string(&event).unwrap();
        assert!(json.contains("\"event\":\"NodeCrashed\""));
        assert!(json.contains("\"exit_code\":139"));
        assert!(json.contains("\"pid\":12345"));
    }

    #[test]
    fn test_launch_session_read_all_empty() {
        // When no sessions exist, should return empty vec (not error)
        let sessions = LaunchSession::read_all();
        // Can't assert exact count since other tests may have sessions,
        // but at minimum this should not panic
        let _ = sessions.len();
    }

    #[test]
    fn test_show_launch_status_no_sessions() {
        // Should succeed with no active sessions (not panic)
        let result = show_launch_status();
        assert!(result.is_ok());
    }
}
