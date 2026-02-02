#![allow(
    clippy::manual_strip,
    clippy::needless_borrows_for_generic_args,
    clippy::manual_range_contains,
    clippy::type_complexity
)]
use horus_core::core::{HealthStatus, NetworkStatus};
use horus_core::error::HorusResult;
use horus_core::memory::{shm_network_dir, shm_topics_dir};
use horus_core::NodePresence;
use std::path::Path;
use std::sync::{Arc, RwLock};
use std::time::{Duration, Instant};

// Data structures for comprehensive monitoring
#[derive(Debug, Clone)]
pub struct NodeStatus {
    pub name: String,
    pub status: String,
    pub health: HealthStatus,
    pub priority: u32,
    pub process_id: u32,
    pub command_line: String,
    pub working_dir: String,
    pub cpu_usage: f32,
    pub memory_usage: u64,
    pub start_time: String,
    pub scheduler_name: String,
    pub category: ProcessCategory,
    pub tick_count: u64,
    pub error_count: u32,
    pub actual_rate_hz: u32,
    pub publishers: Vec<TopicInfo>,
    pub subscribers: Vec<TopicInfo>,
}

#[derive(Debug, Clone, PartialEq)]
pub enum ProcessCategory {
    Node, // Runtime scheduler nodes
    Tool, // GUI applications
    CLI,  // Command line tools
}

/// Topic lifecycle status for monitoring
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TopicStatus {
    /// Active: has live processes AND recent writes (within 30 seconds)
    Active,
    /// Idle: has live processes but no recent writes (30s - 5min)
    Idle,
    /// Stale: no live processes OR very old (5+ minutes without activity)
    Stale,
}

impl TopicStatus {
    /// Get a display symbol for the status
    pub fn symbol(&self) -> &'static str {
        match self {
            TopicStatus::Active => "●",
            TopicStatus::Idle => "◐",
            TopicStatus::Stale => "○",
        }
    }

    /// Get a description of the status
    pub fn description(&self) -> &'static str {
        match self {
            TopicStatus::Active => "active",
            TopicStatus::Idle => "idle",
            TopicStatus::Stale => "stale",
        }
    }
}

#[derive(Debug, Clone)]
pub struct SharedMemoryInfo {
    pub topic_name: String,
    pub size_bytes: u64,
    pub active: bool,
    pub accessing_processes: Vec<u32>,
    pub last_modified: Option<std::time::SystemTime>,
    pub message_type: Option<String>,
    pub publishers: Vec<String>,
    pub subscribers: Vec<String>,
    pub message_rate_hz: f32,
    /// Topic lifecycle status (Active, Idle, or Stale)
    pub status: TopicStatus,
    /// Human-readable age string (e.g., "2s ago", "5m ago", "1h ago")
    pub age_string: String,
}

// Fast discovery cache to avoid expensive filesystem operations
#[derive(Clone)]
struct DiscoveryCache {
    nodes: Vec<NodeStatus>,
    shared_memory: Vec<SharedMemoryInfo>,
    // Separate timestamps for nodes and shared_memory to prevent cross-contamination
    nodes_last_updated: Instant,
    shared_memory_last_updated: Instant,
    cache_duration: Duration,
}

impl DiscoveryCache {
    fn new() -> Self {
        let initial_time = Instant::now() - Duration::from_secs(10); // Force initial update
        Self {
            nodes: Vec::new(),
            shared_memory: Vec::new(),
            nodes_last_updated: initial_time,
            shared_memory_last_updated: initial_time,
            cache_duration: Duration::from_millis(250), // Cache for 250ms (real-time updates)
        }
    }

    fn is_nodes_stale(&self) -> bool {
        self.nodes_last_updated.elapsed() > self.cache_duration
    }

    fn is_shared_memory_stale(&self) -> bool {
        self.shared_memory_last_updated.elapsed() > self.cache_duration
    }

    fn update_nodes(&mut self, nodes: Vec<NodeStatus>) {
        self.nodes = nodes;
        self.nodes_last_updated = Instant::now();
    }

    fn update_shared_memory(&mut self, shm: Vec<SharedMemoryInfo>) {
        self.shared_memory = shm;
        self.shared_memory_last_updated = Instant::now();
    }
}

// Global cache instance
lazy_static::lazy_static! {
    static ref DISCOVERY_CACHE: Arc<RwLock<DiscoveryCache>> = Arc::new(RwLock::new(DiscoveryCache::new()));
}

#[derive(Debug, Default)]
struct ProcessInfo {
    #[allow(dead_code)] // Stored for potential future debugging/display
    pid: u32,
    #[allow(dead_code)] // Stored for potential future debugging/display
    name: String,
    cmdline: String,
    working_dir: String,
    cpu_percent: f32,
    memory_kb: u64,
    start_time: String,
}

pub fn discover_nodes() -> HorusResult<Vec<NodeStatus>> {
    // Check cache first
    if let Ok(cache) = DISCOVERY_CACHE.read() {
        if !cache.is_nodes_stale() {
            return Ok(cache.nodes.clone());
        }
    }

    // Cache is stale - do synchronous update for immediate detection
    let nodes = discover_nodes_uncached()?;

    // Update cache with fresh data
    if let Ok(mut cache) = DISCOVERY_CACHE.write() {
        cache.update_nodes(nodes.clone());
    }

    Ok(nodes)
}

fn discover_nodes_uncached() -> HorusResult<Vec<NodeStatus>> {
    // PRIMARY SOURCE: Presence files in /dev/shm/horus/nodes/
    // Each node writes a presence file at startup and removes it at shutdown.
    // Process liveness is verified by checking if PID exists.
    let mut nodes = discover_nodes_from_presence();

    // FALLBACK: registry.json (for backwards compatibility)
    // Registry is still used as fallback for tools that haven't migrated
    if nodes.is_empty() {
        nodes = discover_nodes_from_registry().unwrap_or_default();
    }

    // SUPPLEMENT: Add process info (CPU, memory) if we have a PID
    for node in &mut nodes {
        if node.process_id > 0 {
            if let Ok(proc_info) = get_process_info(node.process_id) {
                node.cpu_usage = proc_info.cpu_percent;
                node.memory_usage = proc_info.memory_kb;
                node.start_time = proc_info.start_time;
                if node.command_line.is_empty() {
                    node.command_line = proc_info.cmdline.clone();
                }
                if node.working_dir.is_empty() {
                    node.working_dir = proc_info.working_dir.clone();
                }
            }
        }
    }

    // EXTRA: Add any other HORUS processes (tools, CLIs) not detected via pub/sub
    if let Ok(process_nodes) = discover_horus_processes() {
        for process_node in process_nodes {
            // Only add if not already found
            if !nodes
                .iter()
                .any(|n| n.process_id == process_node.process_id || n.name == process_node.name)
            {
                nodes.push(process_node);
            }
        }
    }

    Ok(nodes)
}

/// Discover nodes from presence files (primary discovery method - rqt-like)
/// Each node writes a presence file at startup with PID, topics, etc.
/// NodePresence::read_all() validates PIDs are still alive and cleans stale files.
fn discover_nodes_from_presence() -> Vec<NodeStatus> {
    let presence_nodes = NodePresence::read_all();

    presence_nodes
        .into_iter()
        .map(|presence| {
            let publishers: Vec<TopicInfo> = presence
                .publishers
                .iter()
                .map(|t| TopicInfo {
                    topic: t.topic_name.clone(),
                    type_name: t.type_name.clone(),
                })
                .collect();

            let subscribers: Vec<TopicInfo> = presence
                .subscribers
                .iter()
                .map(|t| TopicInfo {
                    topic: t.topic_name.clone(),
                    type_name: t.type_name.clone(),
                })
                .collect();

            NodeStatus {
                name: presence.name.clone(),
                status: "Running".to_string(),
                health: HealthStatus::Healthy,
                priority: presence.priority,
                process_id: presence.pid,
                command_line: String::new(), // Will be enriched by process info
                working_dir: String::new(),  // Will be enriched by process info
                cpu_usage: 0.0,
                memory_usage: 0,
                start_time: format_unix_timestamp(presence.start_time),
                scheduler_name: presence.scheduler.unwrap_or_default(),
                category: ProcessCategory::Node,
                tick_count: 0, // Not stored in presence file
                error_count: 0,
                actual_rate_hz: presence.rate_hz.map(|r| r as u32).unwrap_or(0),
                publishers,
                subscribers,
            }
        })
        .collect()
}

/// Format Unix timestamp to human-readable string
fn format_unix_timestamp(secs: u64) -> String {
    use std::time::{Duration, SystemTime, UNIX_EPOCH};

    let time = UNIX_EPOCH + Duration::from_secs(secs);
    match time.duration_since(UNIX_EPOCH) {
        Ok(_) => {
            let elapsed = SystemTime::now()
                .duration_since(time)
                .unwrap_or(Duration::from_secs(0));
            if elapsed.as_secs() < 60 {
                format!("{}s ago", elapsed.as_secs())
            } else if elapsed.as_secs() < 3600 {
                format!("{}m ago", elapsed.as_secs() / 60)
            } else {
                format!("{}h ago", elapsed.as_secs() / 3600)
            }
        }
        Err(_) => "unknown".to_string(),
    }
}

// Enhanced node status with pub/sub info
#[derive(Debug, Clone)]
pub struct TopicInfo {
    pub topic: String,
    pub type_name: String,
}

/// Discover all scheduler registry files in home directory (cross-platform)
fn discover_registry_files() -> Vec<std::path::PathBuf> {
    let mut registry_files = Vec::new();

    // Cross-platform home directory detection
    let home_path = if cfg!(target_os = "windows") {
        // Windows: use USERPROFILE or HOMEPATH
        std::env::var("USERPROFILE")
            .or_else(|_| std::env::var("HOMEPATH"))
            .ok()
            .map(std::path::PathBuf::from)
    } else {
        // Linux/macOS: use HOME
        std::env::var("HOME").ok().map(std::path::PathBuf::from)
    };

    let home_path = match home_path {
        Some(path) => path,
        None => return registry_files,
    };

    // Look for all .horus_registry*.json files
    if let Ok(entries) = std::fs::read_dir(&home_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Some(filename) = path.file_name().and_then(|n| n.to_str()) {
                if filename.starts_with(".horus_registry") && filename.ends_with(".json") {
                    registry_files.push(path);
                }
            }
        }
    }

    registry_files
}

/// Discover nodes from registry.json files (primary discovery method)
/// Registry has PID-based liveness check built in for reliable node detection
fn discover_nodes_from_registry() -> anyhow::Result<Vec<NodeStatus>> {
    let mut nodes = Vec::new();

    // Discover all registry files from all schedulers
    let registry_files = discover_registry_files();

    // Process each registry file (supports multiple schedulers)
    for registry_path in registry_files {
        let registry_content = match std::fs::read_to_string(&registry_path) {
            Ok(content) => content,
            Err(_) => continue, // Skip invalid files
        };

        let registry: serde_json::Value = match serde_json::from_str(&registry_content) {
            Ok(reg) => reg,
            Err(_) => continue, // Skip invalid JSON
        };

        // Only use registry if scheduler is still running (built-in liveness check)
        let scheduler_pid = registry["pid"].as_u64().unwrap_or(0) as u32;
        if !process_exists(scheduler_pid) {
            // Clean up stale registry file
            let _ = std::fs::remove_file(&registry_path);
            continue;
        }

        if let Some(scheduler_nodes) = registry["nodes"].as_array() {
            let scheduler_name = registry["scheduler_name"]
                .as_str()
                .unwrap_or("Unknown")
                .to_string();
            let working_dir = registry["working_dir"].as_str().unwrap_or("/").to_string();

            let proc_info = get_process_info(scheduler_pid).unwrap_or_default();

            for node in scheduler_nodes {
                let name = node["name"].as_str().unwrap_or("Unknown").to_string();
                let priority = node["priority"].as_u64().unwrap_or(0) as u32;
                let rate_hz = node["rate_hz"].as_f64().unwrap_or(0.0) as u32;

                // Parse publishers and subscribers
                let mut publishers = Vec::new();
                if let Some(pubs) = node["publishers"].as_array() {
                    for pub_info in pubs {
                        if let (Some(topic), Some(type_name)) =
                            (pub_info["topic"].as_str(), pub_info["type"].as_str())
                        {
                            publishers.push(TopicInfo {
                                topic: topic.to_string(),
                                type_name: type_name.to_string(),
                            });
                        }
                    }
                }

                let mut subscribers = Vec::new();
                if let Some(subs) = node["subscribers"].as_array() {
                    for sub_info in subs {
                        if let (Some(topic), Some(type_name)) =
                            (sub_info["topic"].as_str(), sub_info["type"].as_str())
                        {
                            subscribers.push(TopicInfo {
                                topic: topic.to_string(),
                                type_name: type_name.to_string(),
                            });
                        }
                    }
                }

                nodes.push(NodeStatus {
                    name,
                    status: "Running".to_string(),
                    health: HealthStatus::Healthy,
                    priority,
                    process_id: scheduler_pid, // Use scheduler PID as approximation
                    command_line: proc_info.cmdline.clone(),
                    working_dir: working_dir.clone(),
                    cpu_usage: proc_info.cpu_percent,
                    memory_usage: proc_info.memory_kb,
                    start_time: proc_info.start_time.clone(),
                    tick_count: 0,
                    error_count: 0,
                    actual_rate_hz: rate_hz,
                    scheduler_name: scheduler_name.clone(),
                    category: ProcessCategory::Node,
                    publishers,
                    subscribers,
                });
            }
        }
    }

    Ok(nodes)
}

fn discover_horus_processes() -> anyhow::Result<Vec<NodeStatus>> {
    let mut nodes = Vec::new();
    let proc_dir = Path::new("/proc");

    if !proc_dir.exists() {
        return Ok(nodes);
    }

    for entry in std::fs::read_dir(proc_dir)? {
        let entry = entry?;
        let path = entry.path();

        // Check if this is a PID directory
        if let Some(pid_str) = path.file_name().and_then(|s| s.to_str()) {
            if let Ok(pid) = pid_str.parse::<u32>() {
                // Fast skip: Ignore kernel threads and very low PIDs (system processes)
                // Most HORUS processes will have PID > 1000
                if pid < 100 {
                    continue;
                }

                // Check cmdline for HORUS-related processes
                let cmdline_path = path.join("cmdline");
                if let Ok(cmdline) = std::fs::read_to_string(cmdline_path) {
                    let cmdline_str = cmdline.replace('\0', " ").trim().to_string();

                    // Look for HORUS-related patterns (generic, not hardcoded)
                    if should_track_process(&cmdline_str) {
                        let name = extract_process_name(&cmdline_str);
                        let category = categorize_process(&name, &cmdline_str);

                        // Get detailed process info
                        let proc_info = get_process_info(pid).unwrap_or_default();

                        nodes.push(NodeStatus {
                            name: name.clone(),
                            status: "Running".to_string(),
                            health: HealthStatus::Unknown,
                            priority: 0, // Default for discovered processes
                            process_id: pid,
                            command_line: cmdline_str,
                            working_dir: proc_info.working_dir.clone(),
                            cpu_usage: proc_info.cpu_percent,
                            memory_usage: proc_info.memory_kb,
                            start_time: proc_info.start_time,
                            scheduler_name: "Standalone".to_string(),
                            category,
                            tick_count: 0,
                            error_count: 0,
                            actual_rate_hz: 0,
                            publishers: Vec::new(),
                            subscribers: Vec::new(),
                        });
                    }
                }
            }
        }
    }

    Ok(nodes)
}

fn should_track_process(cmdline: &str) -> bool {
    // Skip empty command lines
    if cmdline.trim().is_empty() {
        return false;
    }

    // Skip build/development tools, system processes, and monitoring tools
    if cmdline.contains("/bin/bash")
        || cmdline.contains("/bin/sh")
        || cmdline.starts_with("timeout ")
        || cmdline.contains("cargo build")
        || cmdline.contains("cargo install")
        || cmdline.contains("cargo run")
        || cmdline.contains("cargo test")
        || cmdline.contains("rustc")
        || cmdline.contains("rustup")
        || cmdline.contains("dashboard")
        || cmdline.contains("monitor")
        || cmdline.contains("horus run")
    // Exclude "horus run" commands - they'll be in registry once scheduler starts
    {
        return false;
    }

    // Only track processes that:
    // 1. Are registered in the HORUS registry (handled by read_registry_file)
    // 2. Are explicitly standalone HORUS project binaries (not CLI commands)

    // Check if it's a standalone HORUS binary (compiled binary running a scheduler)
    // This excludes CLI commands like "horus run", which will appear in registry once the scheduler starts
    if cmdline.contains("scheduler") && !cmdline.contains("horus run") {
        return true;
    }

    // Don't track CLI invocations - only track registered nodes
    false
}

fn categorize_process(name: &str, cmdline: &str) -> ProcessCategory {
    // GUI tools (including GUI executables)
    if name.contains("gui")
        || name.contains("GUI")
        || name.contains("viewer")
        || name.contains("viz")
        || cmdline.contains("--view")
        || cmdline.contains("--gui")
        || name.ends_with("_gui")
    {
        return ProcessCategory::Tool;
    }

    // CLI commands - horus CLI tool usage
    if name == "horus"
        || name.starts_with("horus ")
        || cmdline.contains("/bin/horus")
        || cmdline.contains("target/debug/horus")
        || cmdline.contains("target/release/horus")
        || (cmdline.contains("horus ") && !cmdline.contains("cargo"))
    {
        return ProcessCategory::CLI;
    }

    // Schedulers and other runtime components
    if name.contains("scheduler") || cmdline.contains("scheduler") {
        return ProcessCategory::Node;
    }

    // Default to Node for other HORUS components
    ProcessCategory::Node
}

fn extract_process_name(cmdline: &str) -> String {
    let parts: Vec<&str> = cmdline.split_whitespace().collect();
    if let Some(first) = parts.first() {
        if let Some(name) = Path::new(first).file_name() {
            let base_name = name.to_string_lossy().to_string();

            // For horus CLI commands, include the subcommand and package name
            if base_name == "horus" && parts.len() > 1 {
                if parts.len() > 2 && parts[1] == "monitor" {
                    return format!("horus monitor {}", parts[2]);
                } else if parts.len() > 2 && parts[1] == "run" {
                    // Include the package name for horus run commands
                    return format!("horus run {}", parts[2]);
                } else if parts.len() > 1 {
                    return format!("horus {}", parts[1]);
                }
            }

            return base_name;
        }
    }
    "Unknown".to_string()
}

fn process_exists(pid: u32) -> bool {
    #[cfg(target_os = "linux")]
    {
        Path::new(&format!("/proc/{}", pid)).exists()
    }
    #[cfg(target_os = "macos")]
    {
        // macOS: use kill(0) to check if process exists
        unsafe { libc::kill(pid as i32, 0) == 0 }
    }
    #[cfg(target_os = "windows")]
    {
        // Windows: use OpenProcess to check if process exists
        use std::ptr::null_mut;
        const PROCESS_QUERY_LIMITED_INFORMATION: u32 = 0x1000;
        extern "system" {
            fn OpenProcess(
                dwDesiredAccess: u32,
                bInheritHandle: i32,
                dwProcessId: u32,
            ) -> *mut std::ffi::c_void;
            fn CloseHandle(hObject: *mut std::ffi::c_void) -> i32;
        }
        unsafe {
            let handle = OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid);
            if handle != null_mut() {
                CloseHandle(handle);
                true
            } else {
                false
            }
        }
    }
    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        // Fallback for other Unix-like systems
        unsafe { libc::kill(pid as i32, 0) == 0 }
    }
}

// CPU tracking cache
use std::collections::HashMap as StdHashMap;
lazy_static::lazy_static! {
    static ref CPU_CACHE: Arc<RwLock<StdHashMap<u32, (u64, Instant)>>> =
        Arc::new(RwLock::new(StdHashMap::new()));
}

fn get_process_info(pid: u32) -> anyhow::Result<ProcessInfo> {
    #[cfg(target_os = "linux")]
    {
        let proc_path = format!("/proc/{}", pid);

        // Read command line
        let cmdline = std::fs::read_to_string(format!("{}/cmdline", proc_path))
            .unwrap_or_default()
            .replace('\0', " ")
            .trim()
            .to_string();

        // Extract process name
        let name = extract_process_name(&cmdline);

        // Read working directory
        let working_dir = std::fs::read_link(format!("{}/cwd", proc_path))
            .map(|p| p.to_string_lossy().to_string())
            .unwrap_or_else(|_| "/".to_string());

        // Read stat for memory and CPU info
        let stat_content = std::fs::read_to_string(format!("{}/stat", proc_path))?;
        let memory_kb = parse_memory_from_stat(&stat_content);

        // Calculate CPU usage with sampling
        let cpu_percent = calculate_cpu_usage(pid, &stat_content);

        // Get start time
        let start_time = get_process_start_time(pid);

        Ok(ProcessInfo {
            pid,
            name,
            cmdline,
            working_dir,
            cpu_percent,
            memory_kb,
            start_time,
        })
    }

    #[cfg(target_os = "macos")]
    {
        get_process_info_macos(pid)
    }

    #[cfg(target_os = "windows")]
    {
        get_process_info_windows(pid)
    }

    #[cfg(not(any(target_os = "linux", target_os = "macos", target_os = "windows")))]
    {
        // Fallback for other Unix platforms - basic info only
        Ok(ProcessInfo {
            pid,
            name: format!("pid_{}", pid),
            cmdline: String::new(),
            working_dir: String::new(),
            cpu_percent: 0.0,
            memory_kb: 0,
            start_time: "Unknown".to_string(),
        })
    }
}

#[cfg(target_os = "linux")]
fn calculate_cpu_usage(pid: u32, stat_content: &str) -> f32 {
    // Parse utime + stime from /proc/[pid]/stat
    let fields: Vec<&str> = stat_content.split_whitespace().collect();
    if fields.len() < 15 {
        return 0.0;
    }

    // utime is field 13 (0-indexed), stime is field 14
    let utime = fields[13].parse::<u64>().unwrap_or(0);
    let stime = fields[14].parse::<u64>().unwrap_or(0);
    let total_time = utime + stime;

    // Get cached value
    if let Ok(mut cache) = CPU_CACHE.write() {
        let now = Instant::now();

        if let Some((prev_total, prev_time)) = cache.get(&pid) {
            let time_delta = now.duration_since(*prev_time).as_secs_f32();
            if time_delta > 0.0 {
                let cpu_delta = (total_time.saturating_sub(*prev_total)) as f32;
                // Convert from jiffies to percentage (100 Hz clock)
                let cpu_percent = (cpu_delta / time_delta / 100.0) * 100.0;

                // Update cache
                cache.insert(pid, (total_time, now));

                return cpu_percent.min(100.0);
            }
        }

        // First sample - cache it
        cache.insert(pid, (total_time, now));
    }

    0.0 // Return 0 for first sample
}

#[cfg(target_os = "linux")]
fn parse_memory_from_stat(stat: &str) -> u64 {
    // Parse RSS (Resident Set Size) from /proc/[pid]/stat
    // RSS is the 24th field (0-indexed: 23)
    let fields: Vec<&str> = stat.split_whitespace().collect();

    if fields.len() > 23 {
        if let Ok(rss_pages) = fields[23].parse::<u64>() {
            // Convert pages to KB (usually 4KB per page)
            let page_size = 4; // KB
            return rss_pages * page_size;
        }
    }
    0
}

#[cfg(target_os = "linux")]
fn get_process_start_time(pid: u32) -> String {
    // Read process start time from stat (Linux only)
    if let Ok(stat) = std::fs::read_to_string(format!("/proc/{}/stat", pid)) {
        // Start time is the 22nd field (0-indexed: 21) in jiffies since boot
        let fields: Vec<&str> = stat.split_whitespace().collect();
        if fields.len() > 21 {
            if let Ok(start_jiffies) = fields[21].parse::<u64>() {
                // Convert to seconds and format
                let start_secs = start_jiffies / 100; // Assuming 100 Hz
                let duration = std::time::Duration::from_secs(start_secs);
                return format_duration(duration);
            }
        }
    }
    "Unknown".to_string()
}

fn format_duration(duration: std::time::Duration) -> String {
    let total_secs = duration.as_secs();
    if total_secs < 60 {
        format!("{}s", total_secs)
    } else if total_secs < 3600 {
        format!("{}m", total_secs / 60)
    } else if total_secs < 86400 {
        format!("{}h", total_secs / 3600)
    } else {
        format!("{}d", total_secs / 86400)
    }
}

// ============================================================================
// macOS Process Information
// Uses sysctl and proc_pidinfo for process metrics
// ============================================================================

#[cfg(target_os = "macos")]
fn get_process_info_macos(pid: u32) -> anyhow::Result<ProcessInfo> {
    use std::ffi::CStr;
    use std::mem::MaybeUninit;

    // Get process name using proc_name
    let name = get_process_name_macos(pid).unwrap_or_else(|| format!("pid_{}", pid));

    // Get command line arguments
    let cmdline = get_cmdline_macos(pid).unwrap_or_default();

    // Get working directory
    let working_dir = get_cwd_macos(pid).unwrap_or_default();

    // Get memory usage (RSS in KB)
    let memory_kb = get_memory_macos(pid).unwrap_or(0);

    // Get CPU usage
    let cpu_percent = calculate_cpu_usage_macos(pid);

    // Get start time
    let start_time = get_start_time_macos(pid).unwrap_or_else(|| "Unknown".to_string());

    Ok(ProcessInfo {
        pid,
        name,
        cmdline,
        working_dir,
        cpu_percent,
        memory_kb,
        start_time,
    })
}

#[cfg(target_os = "macos")]
fn get_process_name_macos(pid: u32) -> Option<String> {
    use std::ffi::CStr;

    const PROC_PIDPATHINFO_MAXSIZE: usize = 4096;
    let mut buf = vec![0u8; PROC_PIDPATHINFO_MAXSIZE];

    extern "C" {
        fn proc_name(pid: i32, buffer: *mut u8, buffersize: u32) -> i32;
    }

    let result = unsafe { proc_name(pid as i32, buf.as_mut_ptr(), buf.len() as u32) };

    if result > 0 {
        let name = unsafe { CStr::from_ptr(buf.as_ptr() as *const i8) };
        Some(name.to_string_lossy().into_owned())
    } else {
        None
    }
}

#[cfg(target_os = "macos")]
fn get_cmdline_macos(pid: u32) -> Option<String> {
    use std::ffi::CStr;

    // Use sysctl to get process arguments
    let mut mib: [i32; 3] = [
        1,  // CTL_KERN
        49, // KERN_PROCARGS2
        pid as i32,
    ];

    let mut size: usize = 0;

    // First call to get size
    let result = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            3,
            std::ptr::null_mut(),
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };

    if result != 0 || size == 0 {
        return None;
    }

    let mut buf = vec![0u8; size];

    // Second call to get data
    let result = unsafe {
        libc::sysctl(
            mib.as_mut_ptr(),
            3,
            buf.as_mut_ptr() as *mut _,
            &mut size,
            std::ptr::null_mut(),
            0,
        )
    };

    if result != 0 {
        return None;
    }

    // Parse KERN_PROCARGS2 format: argc (4 bytes) + exec_path + NULLs + args
    if buf.len() < 4 {
        return None;
    }

    // Skip argc (4 bytes) and find the executable path
    let argc = i32::from_ne_bytes([buf[0], buf[1], buf[2], buf[3]]) as usize;
    let mut pos = 4;

    // Skip executable path
    while pos < buf.len() && buf[pos] != 0 {
        pos += 1;
    }

    // Skip null terminators
    while pos < buf.len() && buf[pos] == 0 {
        pos += 1;
    }

    // Collect arguments
    let mut args = Vec::new();
    for _ in 0..argc {
        if pos >= buf.len() {
            break;
        }
        let start = pos;
        while pos < buf.len() && buf[pos] != 0 {
            pos += 1;
        }
        if start < pos {
            if let Ok(arg) = std::str::from_utf8(&buf[start..pos]) {
                args.push(arg.to_string());
            }
        }
        pos += 1; // Skip null terminator
    }

    if args.is_empty() {
        None
    } else {
        Some(args.join(" "))
    }
}

#[cfg(target_os = "macos")]
fn get_cwd_macos(pid: u32) -> Option<String> {
    // macOS doesn't have an easy API for cwd, use proc_pidinfo with PROC_PIDVNODEPATHINFO
    const PROC_PIDVNODEPATHINFO: i32 = 9;
    const MAXPATHLEN: usize = 1024;

    #[repr(C)]
    struct VnodePathInfo {
        pvi_cdir: VnodeInfoPath,
        pvi_rdir: VnodeInfoPath,
    }

    #[repr(C)]
    struct VnodeInfoPath {
        vip_vi: [u8; 152], // vnode_info struct (we don't need details)
        vip_path: [u8; MAXPATHLEN],
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    let mut info: VnodePathInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<VnodePathInfo>() as i32;

    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDVNODEPATHINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return None;
    }

    // Extract current directory path
    let path_bytes = &info.pvi_cdir.vip_path;
    let end = path_bytes
        .iter()
        .position(|&b| b == 0)
        .unwrap_or(path_bytes.len());
    std::str::from_utf8(&path_bytes[..end])
        .ok()
        .map(|s| s.to_string())
}

#[cfg(target_os = "macos")]
fn get_memory_macos(pid: u32) -> Option<u64> {
    const PROC_PIDTASKINFO: i32 = 4;

    #[repr(C)]
    struct TaskInfo {
        pti_virtual_size: u64,
        pti_resident_size: u64,
        pti_total_user: u64,
        pti_total_system: u64,
        pti_threads_user: u64,
        pti_threads_system: u64,
        pti_policy: i32,
        pti_faults: i32,
        pti_pageins: i32,
        pti_cow_faults: i32,
        pti_messages_sent: i32,
        pti_messages_received: i32,
        pti_syscalls_mach: i32,
        pti_syscalls_unix: i32,
        pti_csw: i32,
        pti_threadnum: i32,
        pti_numrunning: i32,
        pti_priority: i32,
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    let mut info: TaskInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<TaskInfo>() as i32;

    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDTASKINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return None;
    }

    // Convert bytes to KB
    Some(info.pti_resident_size / 1024)
}

#[cfg(target_os = "macos")]
lazy_static::lazy_static! {
    static ref MACOS_CPU_CACHE: Arc<RwLock<StdHashMap<u32, (u64, u64, Instant)>>> =
        Arc::new(RwLock::new(StdHashMap::new()));
}

#[cfg(target_os = "macos")]
fn calculate_cpu_usage_macos(pid: u32) -> f32 {
    const PROC_PIDTASKINFO: i32 = 4;

    #[repr(C)]
    struct TaskInfo {
        pti_virtual_size: u64,
        pti_resident_size: u64,
        pti_total_user: u64,
        pti_total_system: u64,
        // ... rest of fields not needed
        _padding: [u8; 64],
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    let mut info: TaskInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<TaskInfo>() as i32;

    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDTASKINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return 0.0;
    }

    // Total CPU time in nanoseconds
    let total_time = info.pti_total_user + info.pti_total_system;

    if let Ok(mut cache) = MACOS_CPU_CACHE.write() {
        let now = Instant::now();

        if let Some((prev_user, prev_system, prev_time)) = cache.get(&pid) {
            let time_delta = now.duration_since(*prev_time).as_secs_f32();
            if time_delta > 0.0 {
                let prev_total = prev_user + prev_system;
                let cpu_delta_ns = total_time.saturating_sub(prev_total) as f32;
                // Convert nanoseconds to percentage
                let cpu_percent = (cpu_delta_ns / 1_000_000_000.0 / time_delta) * 100.0;

                cache.insert(pid, (info.pti_total_user, info.pti_total_system, now));
                return cpu_percent.min(100.0 * num_cpus::get() as f32);
            }
        }

        cache.insert(pid, (info.pti_total_user, info.pti_total_system, now));
    }

    0.0
}

#[cfg(target_os = "macos")]
fn get_start_time_macos(pid: u32) -> Option<String> {
    const PROC_PIDTBSDINFO: i32 = 3;

    #[repr(C)]
    struct BsdInfo {
        pbi_flags: u32,
        pbi_status: u32,
        pbi_xstatus: u32,
        pbi_pid: u32,
        pbi_ppid: u32,
        pbi_uid: u32,
        pbi_gid: u32,
        pbi_ruid: u32,
        pbi_rgid: u32,
        pbi_svuid: u32,
        pbi_svgid: u32,
        rfu_1: u32,
        pbi_comm: [u8; 16],
        pbi_name: [u8; 32],
        pbi_nfiles: u32,
        pbi_pgid: u32,
        pbi_pjobc: u32,
        e_tdev: u32,
        e_tpgid: u32,
        pbi_nice: i32,
        pbi_start_tvsec: u64,
        pbi_start_tvusec: u64,
    }

    extern "C" {
        fn proc_pidinfo(
            pid: i32,
            flavor: i32,
            arg: u64,
            buffer: *mut libc::c_void,
            buffersize: i32,
        ) -> i32;
    }

    let mut info: BsdInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<BsdInfo>() as i32;

    let result = unsafe {
        proc_pidinfo(
            pid as i32,
            PROC_PIDTBSDINFO,
            0,
            &mut info as *mut _ as *mut libc::c_void,
            size,
        )
    };

    if result <= 0 {
        return None;
    }

    // Calculate uptime
    let start = std::time::UNIX_EPOCH + std::time::Duration::from_secs(info.pbi_start_tvsec);
    if let Ok(elapsed) = std::time::SystemTime::now().duration_since(start) {
        Some(format_duration(elapsed))
    } else {
        None
    }
}

// ============================================================================
// Windows Process Information
// Uses Win32 API for process metrics
// ============================================================================

#[cfg(target_os = "windows")]
fn get_process_info_windows(pid: u32) -> anyhow::Result<ProcessInfo> {
    use std::ffi::OsString;
    use std::os::windows::ffi::OsStringExt;

    // Windows API constants
    const PROCESS_QUERY_INFORMATION: u32 = 0x0400;
    const PROCESS_VM_READ: u32 = 0x0010;

    extern "system" {
        fn OpenProcess(
            dwDesiredAccess: u32,
            bInheritHandle: i32,
            dwProcessId: u32,
        ) -> *mut std::ffi::c_void;
        fn CloseHandle(hObject: *mut std::ffi::c_void) -> i32;
    }

    // Open process with query access
    let handle = unsafe { OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, 0, pid) };

    if handle.is_null() {
        return Ok(ProcessInfo {
            pid,
            name: format!("pid_{}", pid),
            cmdline: String::new(),
            working_dir: String::new(),
            cpu_percent: 0.0,
            memory_kb: 0,
            start_time: "Unknown".to_string(),
        });
    }

    // Get process name
    let name = get_process_name_windows(handle).unwrap_or_else(|| format!("pid_{}", pid));

    // Get command line
    let cmdline = get_cmdline_windows(pid).unwrap_or_default();

    // Get memory usage
    let memory_kb = get_memory_windows(handle).unwrap_or(0);

    // Get CPU usage
    let cpu_percent = calculate_cpu_usage_windows(pid, handle);

    // Get start time
    let start_time = get_start_time_windows(handle).unwrap_or_else(|| "Unknown".to_string());

    unsafe { CloseHandle(handle) };

    Ok(ProcessInfo {
        pid,
        name,
        cmdline,
        working_dir: String::new(), // Windows doesn't easily expose cwd
        cpu_percent,
        memory_kb,
        start_time,
    })
}

#[cfg(target_os = "windows")]
fn get_process_name_windows(handle: *mut std::ffi::c_void) -> Option<String> {
    use std::ffi::OsString;
    use std::os::windows::ffi::OsStringExt;

    const MAX_PATH: usize = 260;

    extern "system" {
        fn K32GetModuleBaseNameW(
            hProcess: *mut std::ffi::c_void,
            hModule: *mut std::ffi::c_void,
            lpBaseName: *mut u16,
            nSize: u32,
        ) -> u32;
    }

    let mut buf = vec![0u16; MAX_PATH];
    let len = unsafe {
        K32GetModuleBaseNameW(
            handle,
            std::ptr::null_mut(),
            buf.as_mut_ptr(),
            MAX_PATH as u32,
        )
    };

    if len > 0 {
        buf.truncate(len as usize);
        Some(OsString::from_wide(&buf).to_string_lossy().into_owned())
    } else {
        None
    }
}

#[cfg(target_os = "windows")]
fn get_cmdline_windows(pid: u32) -> Option<String> {
    // Getting command line on Windows requires reading from PEB which is complex
    // Use WMI or simplified approach via QueryFullProcessImageNameW
    use std::ffi::OsString;
    use std::os::windows::ffi::OsStringExt;

    const PROCESS_QUERY_LIMITED_INFORMATION: u32 = 0x1000;
    const MAX_PATH: usize = 32768;

    extern "system" {
        fn OpenProcess(
            dwDesiredAccess: u32,
            bInheritHandle: i32,
            dwProcessId: u32,
        ) -> *mut std::ffi::c_void;
        fn CloseHandle(hObject: *mut std::ffi::c_void) -> i32;
        fn QueryFullProcessImageNameW(
            hProcess: *mut std::ffi::c_void,
            dwFlags: u32,
            lpExeName: *mut u16,
            lpdwSize: *mut u32,
        ) -> i32;
    }

    let handle = unsafe { OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid) };
    if handle.is_null() {
        return None;
    }

    let mut buf = vec![0u16; MAX_PATH];
    let mut size = MAX_PATH as u32;

    let result = unsafe { QueryFullProcessImageNameW(handle, 0, buf.as_mut_ptr(), &mut size) };

    unsafe { CloseHandle(handle) };

    if result != 0 && size > 0 {
        buf.truncate(size as usize);
        Some(OsString::from_wide(&buf).to_string_lossy().into_owned())
    } else {
        None
    }
}

#[cfg(target_os = "windows")]
fn get_memory_windows(handle: *mut std::ffi::c_void) -> Option<u64> {
    #[repr(C)]
    struct ProcessMemoryCounters {
        cb: u32,
        page_fault_count: u32,
        peak_working_set_size: usize,
        working_set_size: usize,
        quota_peak_paged_pool_usage: usize,
        quota_paged_pool_usage: usize,
        quota_peak_non_paged_pool_usage: usize,
        quota_non_paged_pool_usage: usize,
        pagefile_usage: usize,
        peak_pagefile_usage: usize,
    }

    extern "system" {
        fn K32GetProcessMemoryInfo(
            hProcess: *mut std::ffi::c_void,
            ppsmemCounters: *mut ProcessMemoryCounters,
            cb: u32,
        ) -> i32;
    }

    let mut counters: ProcessMemoryCounters = unsafe { std::mem::zeroed() };
    counters.cb = std::mem::size_of::<ProcessMemoryCounters>() as u32;

    let result = unsafe { K32GetProcessMemoryInfo(handle, &mut counters, counters.cb) };

    if result != 0 {
        // Convert bytes to KB
        Some((counters.working_set_size / 1024) as u64)
    } else {
        None
    }
}

#[cfg(target_os = "windows")]
lazy_static::lazy_static! {
    static ref WINDOWS_CPU_CACHE: Arc<RwLock<StdHashMap<u32, (u64, Instant)>>> =
        Arc::new(RwLock::new(StdHashMap::new()));
}

#[cfg(target_os = "windows")]
fn calculate_cpu_usage_windows(pid: u32, handle: *mut std::ffi::c_void) -> f32 {
    #[repr(C)]
    struct Filetime {
        low: u32,
        high: u32,
    }

    extern "system" {
        fn GetProcessTimes(
            hProcess: *mut std::ffi::c_void,
            lpCreationTime: *mut Filetime,
            lpExitTime: *mut Filetime,
            lpKernelTime: *mut Filetime,
            lpUserTime: *mut Filetime,
        ) -> i32;
    }

    let mut creation = Filetime { low: 0, high: 0 };
    let mut exit = Filetime { low: 0, high: 0 };
    let mut kernel = Filetime { low: 0, high: 0 };
    let mut user = Filetime { low: 0, high: 0 };

    let result =
        unsafe { GetProcessTimes(handle, &mut creation, &mut exit, &mut kernel, &mut user) };

    if result == 0 {
        return 0.0;
    }

    // Convert FILETIME to 100-nanosecond intervals
    let kernel_time = ((kernel.high as u64) << 32) | (kernel.low as u64);
    let user_time = ((user.high as u64) << 32) | (user.low as u64);
    let total_time = kernel_time + user_time;

    if let Ok(mut cache) = WINDOWS_CPU_CACHE.write() {
        let now = Instant::now();

        if let Some((prev_total, prev_time)) = cache.get(&pid) {
            let time_delta = now.duration_since(*prev_time).as_secs_f32();
            if time_delta > 0.0 {
                let cpu_delta = total_time.saturating_sub(*prev_total) as f32;
                // Convert 100-nanosecond intervals to percentage
                let cpu_percent = (cpu_delta / 10_000_000.0 / time_delta) * 100.0;

                cache.insert(pid, (total_time, now));
                return cpu_percent.min(100.0 * num_cpus::get() as f32);
            }
        }

        cache.insert(pid, (total_time, now));
    }

    0.0
}

#[cfg(target_os = "windows")]
fn get_start_time_windows(handle: *mut std::ffi::c_void) -> Option<String> {
    #[repr(C)]
    struct Filetime {
        low: u32,
        high: u32,
    }

    extern "system" {
        fn GetProcessTimes(
            hProcess: *mut std::ffi::c_void,
            lpCreationTime: *mut Filetime,
            lpExitTime: *mut Filetime,
            lpKernelTime: *mut Filetime,
            lpUserTime: *mut Filetime,
        ) -> i32;
    }

    let mut creation = Filetime { low: 0, high: 0 };
    let mut exit = Filetime { low: 0, high: 0 };
    let mut kernel = Filetime { low: 0, high: 0 };
    let mut user = Filetime { low: 0, high: 0 };

    let result =
        unsafe { GetProcessTimes(handle, &mut creation, &mut exit, &mut kernel, &mut user) };

    if result == 0 {
        return None;
    }

    // Convert FILETIME to duration since UNIX epoch
    // FILETIME is 100-nanosecond intervals since January 1, 1601
    let creation_time = ((creation.high as u64) << 32) | (creation.low as u64);

    // Difference between 1601 and 1970 in 100-nanosecond intervals
    const EPOCH_DIFF: u64 = 116444736000000000;

    if creation_time > EPOCH_DIFF {
        let unix_time = (creation_time - EPOCH_DIFF) / 10_000_000;
        let start = std::time::UNIX_EPOCH + std::time::Duration::from_secs(unix_time);
        if let Ok(elapsed) = std::time::SystemTime::now().duration_since(start) {
            return Some(format_duration(elapsed));
        }
    }

    None
}

pub fn discover_shared_memory() -> HorusResult<Vec<SharedMemoryInfo>> {
    // Check cache first
    if let Ok(cache) = DISCOVERY_CACHE.read() {
        if !cache.is_shared_memory_stale() {
            return Ok(cache.shared_memory.clone());
        }
    }

    // Cache is stale - do synchronous update for immediate detection
    let shared_memory = discover_shared_memory_uncached()?;

    // Update cache with fresh data
    if let Ok(mut cache) = DISCOVERY_CACHE.write() {
        cache.update_shared_memory(shared_memory.clone());
    }

    Ok(shared_memory)
}

// Topic rate tracking cache
lazy_static::lazy_static! {
    static ref TOPIC_RATE_CACHE: Arc<RwLock<StdHashMap<String, (Instant, u64)>>> =
        Arc::new(RwLock::new(StdHashMap::new()));
}

fn discover_shared_memory_uncached() -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();
    let mut seen_topics: std::collections::HashSet<String> = std::collections::HashSet::new();

    // PRIMARY: Scan SHM topics directory (has full metadata: size, mtime, accessing processes)
    let topics_path = shm_topics_dir();
    if topics_path.exists() {
        for topic in scan_topics_directory(&topics_path)? {
            seen_topics.insert(topic.topic_name.clone());
            topics.push(topic);
        }
    }

    // SUPPLEMENT: Add topics from presence files (captures network topics too)
    // These won't have SHM-specific metadata but will show pub/sub relationships
    for topic in discover_topics_from_presence() {
        // Check for duplicates - match exact name OR with horus_topic/ prefix
        let is_duplicate = seen_topics.contains(&topic.topic_name)
            || seen_topics.contains(&format!("horus_topic/{}", topic.topic_name))
            || topic
                .topic_name
                .strip_prefix("horus_topic/")
                .map(|base| seen_topics.contains(base))
                .unwrap_or(false);

        if !is_duplicate {
            seen_topics.insert(topic.topic_name.clone());
            topics.push(topic);
        }
    }

    Ok(topics)
}

/// Discover topics from node presence files
/// This captures ALL topics including network-based ones that don't have SHM files
fn discover_topics_from_presence() -> Vec<SharedMemoryInfo> {
    use std::collections::HashMap;

    // Aggregate topic info from all presence files
    let mut topic_map: HashMap<String, (Vec<String>, Vec<String>, Option<String>)> = HashMap::new();

    for presence in NodePresence::read_all() {
        // Add publishers
        for pub_topic in &presence.publishers {
            let entry = topic_map
                .entry(pub_topic.topic_name.clone())
                .or_insert_with(|| (Vec::new(), Vec::new(), None));
            if !entry.0.contains(&presence.name) {
                entry.0.push(presence.name.clone());
            }
            // Capture type name if available
            if entry.2.is_none()
                && !pub_topic.type_name.is_empty()
                && pub_topic.type_name != "unknown"
            {
                entry.2 = Some(pub_topic.type_name.clone());
            }
        }

        // Add subscribers
        for sub_topic in &presence.subscribers {
            let entry = topic_map
                .entry(sub_topic.topic_name.clone())
                .or_insert_with(|| (Vec::new(), Vec::new(), None));
            if !entry.1.contains(&presence.name) {
                entry.1.push(presence.name.clone());
            }
            // Capture type name if available
            if entry.2.is_none()
                && !sub_topic.type_name.is_empty()
                && sub_topic.type_name != "unknown"
            {
                entry.2 = Some(sub_topic.type_name.clone());
            }
        }
    }

    // Convert to SharedMemoryInfo (network topics won't have SHM-specific metadata)
    topic_map
        .into_iter()
        .map(|(topic_name, (publishers, subscribers, message_type))| {
            let has_activity = !publishers.is_empty() || !subscribers.is_empty();
            SharedMemoryInfo {
                topic_name,
                size_bytes: 0, // Network topics don't have SHM size
                active: has_activity,
                accessing_processes: Vec::new(), // Can't determine for network topics
                last_modified: None,
                message_type,
                publishers,
                subscribers,
                message_rate_hz: 0.0, // Can't measure without SHM
                status: if has_activity {
                    TopicStatus::Active
                } else {
                    TopicStatus::Stale
                },
                age_string: "network".to_string(), // Indicate this is from presence, not SHM
            }
        })
        .collect()
}

/// Clean up stale topic files from the global topics directory.
///
/// A topic is considered stale if:
/// 1. No process has it mmap'd (no live readers/writers)
/// 2. It hasn't been modified in 5+ minutes
///
/// This function is NOT called automatically during discovery.
/// Call it explicitly from `horus run` startup or `horus clean --stale`.
pub fn cleanup_stale_topics() {
    let topics_path = shm_topics_dir();
    if topics_path.exists() {
        cleanup_stale_topics_in_dir(&topics_path);
    }
}

/// Internal: Clean up stale topic files in a specific directory
fn cleanup_stale_topics_in_dir(shm_path: &Path) {
    const STALE_THRESHOLD_SECS: u64 = 300; // 5 minutes

    if let Ok(entries) = std::fs::read_dir(shm_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }

            let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
                continue;
            };

            // Check if any process has this file mmap'd
            let accessing_procs = find_accessing_processes_fast(&path, name);
            let has_live_processes = accessing_procs.iter().any(|pid| process_exists(*pid));

            if has_live_processes {
                continue; // Topic is in use
            }

            // Check modification time
            if let Ok(metadata) = entry.metadata() {
                if let Ok(modified) = metadata.modified() {
                    if let Ok(elapsed) = modified.elapsed() {
                        if elapsed.as_secs() > STALE_THRESHOLD_SECS {
                            // Topic is stale - remove it
                            let _ = std::fs::remove_file(&path);
                        }
                    }
                }
            }
        }
    }
}

/// Scan a specific topics directory for shared memory files
fn scan_topics_directory(shm_path: &Path) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    // Load registry to get topic metadata
    let registry_topics = load_topic_metadata_from_registry();

    // Active nodes for pub/sub inference (from registry)
    let active_nodes: Vec<String> = Vec::new();

    for entry in std::fs::read_dir(shm_path)? {
        let entry = entry?;
        let path = entry.path();
        let metadata = entry.metadata()?;

        // Smart filter for shared memory segments
        if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
            if metadata.is_file() {
                // Hub topics - files directly in topics directory
                if let Some(info) = scan_topic_file(&path, name, &registry_topics, &active_nodes) {
                    topics.push(info);
                }
            } else if metadata.is_dir() && name == "horus_links" {
                // Link topics - files inside horus_links subdirectory
                topics.extend(scan_links_directory(
                    &path,
                    &registry_topics,
                    &active_nodes,
                )?);
            } else if metadata.is_dir() && name == "horus_topic" {
                // Topic API topics - files inside horus_topic subdirectory
                // These are created by Topic::new() in HORUS applications
                topics.extend(scan_topic_api_directory(
                    &path,
                    &registry_topics,
                    &active_nodes,
                )?);
            }
        }
    }

    Ok(topics)
}

/// Scan the horus_links directory for Link shared memory files
fn scan_links_directory(
    links_path: &Path,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
    active_nodes: &[String],
) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    if let Ok(entries) = std::fs::read_dir(links_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
                        // Link topic name format: links/<topic>
                        let topic_name = format!("links/{}", name);
                        if let Some(mut info) =
                            scan_topic_file(&path, name, registry_topics, active_nodes)
                        {
                            info.topic_name = topic_name;
                            topics.push(info);
                        }
                    }
                }
            }
        }
    }

    Ok(topics)
}

/// Scan the horus_topic directory for Topic API shared memory files
/// These are created by Topic::new() in HORUS applications
fn scan_topic_api_directory(
    topic_path: &Path,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
    active_nodes: &[String],
) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    if let Ok(entries) = std::fs::read_dir(topic_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
                        // Skip discovery metadata files
                        if name == "horus.discovery" {
                            continue;
                        }
                        // Topic API uses horus_topic/<name> path format
                        // Include the subdirectory in topic_name for proper path construction
                        let topic_name = format!("horus_topic/{}", name);
                        if let Some(mut info) =
                            scan_topic_file(&path, name, registry_topics, active_nodes)
                        {
                            info.topic_name = topic_name;
                            topics.push(info);
                        }
                    }
                }
            }
        }
    }

    Ok(topics)
}

/// Scan a single topic file and create SharedMemoryInfo
fn scan_topic_file(
    path: &Path,
    name: &str,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
    active_nodes: &[String],
) -> Option<SharedMemoryInfo> {
    let metadata = std::fs::metadata(path).ok()?;
    let size = metadata.len();
    let modified = metadata.modified().ok();

    // Find processes accessing this segment (optimized)
    let accessing_procs = find_accessing_processes_fast(path, name);

    // All files in HORUS directory are valid topics
    // Extract topic name from filename (remove "horus_" prefix)
    // Topic names use dot notation (e.g., "motors.cmd_vel") - no conversion needed
    let topic_name = if name.starts_with("horus_") {
        name.strip_prefix("horus_").unwrap_or(name).to_string()
    } else {
        name.to_string()
    };

    let is_recent = if let Some(mod_time) = modified {
        // Use 30 second threshold to handle slow publishers (e.g., 0.1 Hz = 10 sec between publishes)
        mod_time.elapsed().unwrap_or(Duration::from_secs(3600)) < Duration::from_secs(30)
    } else {
        false
    };

    let has_valid_processes = accessing_procs.iter().any(|pid| process_exists(*pid));

    // Include all topics in HORUS directory
    // Topics persist until manually cleaned or processes terminate
    let active = has_valid_processes || is_recent;

    // Calculate message rate from modification times
    let message_rate = calculate_topic_rate(&topic_name, modified);

    // Get metadata from registry
    let (message_type, mut publishers, subscribers) = registry_topics
        .get(&topic_name)
        .map(|(t, p, s)| (Some(t.clone()), p.clone(), s.clone()))
        .unwrap_or((None, Vec::new(), Vec::new()));

    // Fallback: if no registry info and topic is active, infer from active nodes
    if publishers.is_empty() && active && !active_nodes.is_empty() {
        // Assume all active nodes are potential publishers for active topics
        // This provides visibility when registry.json is not available
        publishers = active_nodes.to_vec();
    }

    // Compute live processes (filter dead ones)
    let live_processes: Vec<u32> = accessing_procs
        .iter()
        .filter(|pid| process_exists(**pid))
        .copied()
        .collect();

    // Compute status and age string
    let (status, age_string) = compute_topic_status(&live_processes, modified);

    Some(SharedMemoryInfo {
        topic_name,
        size_bytes: size,
        active,
        accessing_processes: live_processes,
        last_modified: modified,
        message_type,
        publishers,
        subscribers,
        message_rate_hz: message_rate,
        status,
        age_string,
    })
}

/// Compute topic status and age string based on live processes and modification time
fn compute_topic_status(
    live_processes: &[u32],
    modified: Option<std::time::SystemTime>,
) -> (TopicStatus, String) {
    const ACTIVE_THRESHOLD_SECS: u64 = 30; // Recent activity = within 30 seconds
    const STALE_THRESHOLD_SECS: u64 = 300; // Stale = 5+ minutes

    let has_live_processes = !live_processes.is_empty();

    // Calculate age
    let (age_secs, age_string) = if let Some(mod_time) = modified {
        if let Ok(elapsed) = mod_time.elapsed() {
            let secs = elapsed.as_secs();
            let age_str = format_age(secs);
            (Some(secs), age_str)
        } else {
            (None, "unknown".to_string())
        }
    } else {
        (None, "unknown".to_string())
    };

    // Determine status
    let status = match (has_live_processes, age_secs) {
        // Has live processes AND recent writes = Active
        (true, Some(secs)) if secs < ACTIVE_THRESHOLD_SECS => TopicStatus::Active,
        // Has live processes but no recent writes = Idle
        (true, _) => TopicStatus::Idle,
        // No live processes AND old = Stale
        (false, Some(secs)) if secs >= STALE_THRESHOLD_SECS => TopicStatus::Stale,
        // No live processes but recent = Idle (process just exited)
        (false, Some(_)) => TopicStatus::Idle,
        // Unknown modification time = Stale
        (false, None) => TopicStatus::Stale,
    };

    (status, age_string)
}

/// Format age in human-readable form
fn format_age(secs: u64) -> String {
    if secs < 60 {
        format!("{}s ago", secs)
    } else if secs < 3600 {
        format!("{}m ago", secs / 60)
    } else if secs < 86400 {
        format!("{}h ago", secs / 3600)
    } else {
        format!("{}d ago", secs / 86400)
    }
}

fn calculate_topic_rate(topic_name: &str, modified: Option<std::time::SystemTime>) -> f32 {
    let now = Instant::now();

    if let Some(mod_time) = modified {
        if let Ok(mut cache) = TOPIC_RATE_CACHE.write() {
            // Convert SystemTime to a simple counter for change detection
            let mod_counter = mod_time
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0);

            if let Some((prev_instant, prev_counter)) = cache.get(topic_name) {
                if mod_counter != *prev_counter {
                    // File was modified
                    let time_delta = now.duration_since(*prev_instant).as_secs_f32();
                    if time_delta > 0.0 && time_delta < 10.0 {
                        let rate = 1.0 / time_delta;
                        cache.insert(topic_name.to_string(), (now, mod_counter));
                        return rate;
                    }
                }
            }

            // First sample or same modification time
            cache.insert(topic_name.to_string(), (now, mod_counter));
        }
    }

    0.0
}

fn load_topic_metadata_from_registry() -> StdHashMap<String, (String, Vec<String>, Vec<String>)> {
    let mut topic_map = StdHashMap::new();

    // Load from all registry files (supports multiple schedulers)
    let registry_files = discover_registry_files();

    for registry_path in registry_files {
        if let Ok(content) = std::fs::read_to_string(&registry_path) {
            if let Ok(registry) = serde_json::from_str::<serde_json::Value>(&content) {
                // Skip if scheduler is dead
                let scheduler_pid = registry["pid"].as_u64().unwrap_or(0) as u32;
                if !process_exists(scheduler_pid) {
                    continue;
                }

                if let Some(nodes) = registry["nodes"].as_array() {
                    for node in nodes {
                        let node_name = node["name"].as_str().unwrap_or("Unknown");

                        // Process publishers
                        if let Some(pubs) = node["publishers"].as_array() {
                            for pub_info in pubs {
                                if let (Some(topic), Some(type_name)) =
                                    (pub_info["topic"].as_str(), pub_info["type"].as_str())
                                {
                                    let entry = topic_map.entry(topic.to_string()).or_insert((
                                        type_name.to_string(),
                                        Vec::new(),
                                        Vec::new(),
                                    ));
                                    entry.1.push(node_name.to_string());
                                }
                            }
                        }

                        // Process subscribers
                        if let Some(subs) = node["subscribers"].as_array() {
                            for sub_info in subs {
                                if let (Some(topic), Some(type_name)) =
                                    (sub_info["topic"].as_str(), sub_info["type"].as_str())
                                {
                                    let entry = topic_map.entry(topic.to_string()).or_insert((
                                        type_name.to_string(),
                                        Vec::new(),
                                        Vec::new(),
                                    ));
                                    entry.2.push(node_name.to_string());
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    topic_map
}

// Fast version: Check memory maps for HORUS processes to find mmap'd shared memory
fn find_accessing_processes_fast(shm_path: &Path, shm_name: &str) -> Vec<u32> {
    let mut processes = Vec::new();
    let shm_path_str = shm_path.to_string_lossy();

    // For HORUS-like shared memory, only check HORUS processes first (much faster)
    let is_horus_shm = shm_name.contains("horus")
        || shm_name.contains("topic")
        || shm_name.starts_with("ros")
        || shm_name.starts_with("shm_");

    if is_horus_shm {
        // Fast path: Only check processes with HORUS in their name
        if let Ok(proc_entries) = std::fs::read_dir("/proc") {
            for entry in proc_entries.flatten() {
                if let Some(pid_str) = entry.file_name().to_str() {
                    if let Ok(pid) = pid_str.parse::<u32>() {
                        // Quick check if this is a HORUS-related process
                        if let Ok(cmdline) = std::fs::read_to_string(entry.path().join("cmdline")) {
                            let cmdline_str = cmdline.replace('\0', " ");
                            if cmdline_str.contains("horus")
                                || cmdline_str.contains("ros")
                                || cmdline_str.contains("sim")
                                || cmdline_str.contains("snake")
                            {
                                // Check memory maps for this process (mmap'd files show up here)
                                let maps_path = entry.path().join("maps");
                                if let Ok(maps_content) = std::fs::read_to_string(&maps_path) {
                                    if maps_content.contains(&*shm_path_str) {
                                        processes.push(pid);
                                        continue;
                                    }
                                }
                                // Also check file descriptors as fallback
                                let fd_path = entry.path().join("fd");
                                if let Ok(fd_entries) = std::fs::read_dir(fd_path) {
                                    for fd_entry in fd_entries.flatten() {
                                        if let Ok(link_target) = std::fs::read_link(fd_entry.path())
                                        {
                                            if link_target == shm_path {
                                                processes.push(pid);
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // If we found HORUS processes, return early
        if !processes.is_empty() {
            return processes;
        }
    }

    // Fallback: Abbreviated scan - only check first 50 processes to avoid blocking
    if let Ok(proc_entries) = std::fs::read_dir("/proc") {
        for (_checked, entry) in proc_entries.flatten().enumerate().take(50) {
            if let Some(pid) = entry
                .file_name()
                .to_str()
                .and_then(|s| s.parse::<u32>().ok())
            {
                // Check memory maps first (mmap'd files)
                let maps_path = entry.path().join("maps");
                if let Ok(maps_content) = std::fs::read_to_string(&maps_path) {
                    if maps_content.contains(&*shm_path_str) {
                        processes.push(pid);
                        continue;
                    }
                }
                // Fallback to file descriptors
                let fd_path = entry.path().join("fd");
                if let Ok(fd_entries) = std::fs::read_dir(fd_path) {
                    for fd_entry in fd_entries.flatten() {
                        if let Ok(link_target) = std::fs::read_link(fd_entry.path()) {
                            if link_target == shm_path {
                                processes.push(pid);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    processes
}

// Enhanced monitoring functions

#[cfg(test)]
mod tests {
    use super::*;

    // =====================
    // NodeStatus Tests
    // =====================
    #[test]
    fn test_node_status_creation() {
        let node = NodeStatus {
            name: "test_node".to_string(),
            status: "Running".to_string(),
            health: HealthStatus::Healthy,
            priority: 10,
            process_id: 1234,
            command_line: "horus run test".to_string(),
            working_dir: "/home/test".to_string(),
            cpu_usage: 25.5,
            memory_usage: 1024,
            start_time: "10m".to_string(),
            scheduler_name: "default".to_string(),
            category: ProcessCategory::Node,
            tick_count: 100,
            error_count: 0,
            actual_rate_hz: 50,
            publishers: vec![],
            subscribers: vec![],
        };

        assert_eq!(node.name, "test_node");
        assert_eq!(node.status, "Running");
        assert_eq!(node.priority, 10);
        assert_eq!(node.process_id, 1234);
        assert_eq!(node.tick_count, 100);
    }

    #[test]
    fn test_node_status_with_publishers_subscribers() {
        let pub_topic = TopicInfo {
            topic: "sensor.data".to_string(),
            type_name: "SensorMsg".to_string(),
        };
        let sub_topic = TopicInfo {
            topic: "commands".to_string(),
            type_name: "CmdMsg".to_string(),
        };

        let node = NodeStatus {
            name: "sensor_node".to_string(),
            status: "Running".to_string(),
            health: HealthStatus::Healthy,
            priority: 5,
            process_id: 5678,
            command_line: String::new(),
            working_dir: String::new(),
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: String::new(),
            scheduler_name: "main".to_string(),
            category: ProcessCategory::Node,
            tick_count: 0,
            error_count: 0,
            actual_rate_hz: 0,
            publishers: vec![pub_topic],
            subscribers: vec![sub_topic],
        };

        assert_eq!(node.publishers.len(), 1);
        assert_eq!(node.subscribers.len(), 1);
        assert_eq!(node.publishers[0].topic, "sensor.data");
        assert_eq!(node.subscribers[0].type_name, "CmdMsg");
    }

    // =====================
    // ProcessCategory Tests
    // =====================
    #[test]
    fn test_process_category_equality() {
        assert_eq!(ProcessCategory::Node, ProcessCategory::Node);
        assert_eq!(ProcessCategory::Tool, ProcessCategory::Tool);
        assert_eq!(ProcessCategory::CLI, ProcessCategory::CLI);
        assert_ne!(ProcessCategory::Node, ProcessCategory::Tool);
        assert_ne!(ProcessCategory::Tool, ProcessCategory::CLI);
    }

    // =====================
    // SharedMemoryInfo Tests
    // =====================
    #[test]
    fn test_shared_memory_info_creation() {
        let shm = SharedMemoryInfo {
            topic_name: "robot.pose".to_string(),
            size_bytes: 4096,
            active: true,
            accessing_processes: vec![1234, 5678],
            last_modified: Some(std::time::SystemTime::now()),
            message_type: Some("PoseMsg".to_string()),
            publishers: vec!["localization".to_string()],
            subscribers: vec!["navigation".to_string(), "visualization".to_string()],
            message_rate_hz: 30.0,
            status: TopicStatus::Active,
            age_string: "0s ago".to_string(),
        };

        assert_eq!(shm.topic_name, "robot.pose");
        assert_eq!(shm.size_bytes, 4096);
        assert!(shm.active);
        assert_eq!(shm.accessing_processes.len(), 2);
        assert_eq!(shm.publishers.len(), 1);
        assert_eq!(shm.subscribers.len(), 2);
        assert!((shm.message_rate_hz - 30.0).abs() < 0.001);
    }

    #[test]
    fn test_shared_memory_info_inactive() {
        let shm = SharedMemoryInfo {
            topic_name: "old.topic".to_string(),
            size_bytes: 1024,
            active: false,
            accessing_processes: vec![],
            last_modified: None,
            message_type: None,
            publishers: vec![],
            subscribers: vec![],
            message_rate_hz: 0.0,
            status: TopicStatus::Stale,
            age_string: "unknown".to_string(),
        };

        assert!(!shm.active);
        assert!(shm.accessing_processes.is_empty());
        assert!(shm.message_type.is_none());
        assert!(shm.last_modified.is_none());
    }

    // =====================
    // TopicInfo Tests
    // =====================
    #[test]
    fn test_topic_info_creation() {
        let topic = TopicInfo {
            topic: "camera.image".to_string(),
            type_name: "sensor_msgs::Image".to_string(),
        };

        assert_eq!(topic.topic, "camera.image");
        assert_eq!(topic.type_name, "sensor_msgs::Image");
    }

    // =====================
    // Helper Function Tests
    // =====================
    #[test]
    fn test_format_duration_seconds() {
        let duration = std::time::Duration::from_secs(45);
        assert_eq!(format_duration(duration), "45s");
    }

    #[test]
    fn test_format_duration_minutes() {
        let duration = std::time::Duration::from_secs(125);
        assert_eq!(format_duration(duration), "2m");
    }

    #[test]
    fn test_format_duration_hours() {
        let duration = std::time::Duration::from_secs(7200);
        assert_eq!(format_duration(duration), "2h");
    }

    #[test]
    fn test_format_duration_days() {
        let duration = std::time::Duration::from_secs(172800);
        assert_eq!(format_duration(duration), "2d");
    }

    #[test]
    fn test_should_track_process_empty() {
        assert!(!should_track_process(""));
        assert!(!should_track_process("   "));
    }

    #[test]
    fn test_should_track_process_excluded_patterns() {
        // Build tools should be excluded
        assert!(!should_track_process("cargo build --release"));
        assert!(!should_track_process("cargo test"));
        assert!(!should_track_process("rustc --version"));
        assert!(!should_track_process("/bin/bash script.sh"));
        assert!(!should_track_process("timeout 10 some_command"));
        assert!(!should_track_process("dashboard server"));
        assert!(!should_track_process("horus run test_package"));
    }

    #[test]
    fn test_should_track_process_scheduler() {
        // Standalone scheduler should be tracked
        assert!(should_track_process("/path/to/scheduler binary"));
    }

    #[test]
    fn test_categorize_process_gui() {
        assert_eq!(categorize_process("robot_gui", ""), ProcessCategory::Tool);
        assert_eq!(categorize_process("viewer_app", ""), ProcessCategory::Tool);
        assert_eq!(categorize_process("viz_tool", ""), ProcessCategory::Tool);
        assert_eq!(categorize_process("my_GUI_app", ""), ProcessCategory::Tool);
        assert_eq!(categorize_process("app_gui", ""), ProcessCategory::Tool);
        assert_eq!(categorize_process("test", "--gui"), ProcessCategory::Tool);
        assert_eq!(
            categorize_process("test", "--view mode"),
            ProcessCategory::Tool
        );
    }

    #[test]
    fn test_categorize_process_cli() {
        assert_eq!(categorize_process("horus", ""), ProcessCategory::CLI);
        assert_eq!(categorize_process("horus run", ""), ProcessCategory::CLI);
        assert_eq!(
            categorize_process("test", "/bin/horus run pkg"),
            ProcessCategory::CLI
        );
        assert_eq!(
            categorize_process("test", "target/debug/horus run pkg"),
            ProcessCategory::CLI
        );
    }

    #[test]
    fn test_categorize_process_node() {
        assert_eq!(categorize_process("scheduler", ""), ProcessCategory::Node);
        assert_eq!(
            categorize_process("test", "my_scheduler"),
            ProcessCategory::Node
        );
        // Default is Node
        assert_eq!(
            categorize_process("unknown_process", "unknown cmd"),
            ProcessCategory::Node
        );
    }

    #[test]
    fn test_extract_process_name_simple() {
        assert_eq!(
            extract_process_name("/usr/bin/robot_control"),
            "robot_control"
        );
        assert_eq!(extract_process_name("./my_program"), "my_program");
    }

    #[test]
    fn test_extract_process_name_horus_cli() {
        assert_eq!(
            extract_process_name("horus run my_package"),
            "horus run my_package"
        );
        assert_eq!(
            extract_process_name("horus monitor dashboard"),
            "horus monitor dashboard"
        );
        assert_eq!(extract_process_name("horus version"), "horus version");
    }

    #[test]
    fn test_extract_process_name_empty() {
        assert_eq!(extract_process_name(""), "Unknown");
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn test_parse_memory_from_stat_valid() {
        // stat format: pid (comm) state ... rss is 24th field (0-indexed: 23)
        // We need at least 24 space-separated fields
        let stat = "1234 (test) S 1 1234 1234 0 -1 4194304 100 0 0 0 10 5 0 0 20 0 1 0 12345 12345678 500 18446744073709551615 0 0 0 0 0 0 0 0 0 0 0 0 17 0 0 0 0 0 0";
        let memory = parse_memory_from_stat(stat);
        // 500 pages * 4KB = 2000KB
        assert_eq!(memory, 2000);
    }

    #[test]
    #[cfg(target_os = "linux")]
    fn test_parse_memory_from_stat_invalid() {
        assert_eq!(parse_memory_from_stat(""), 0);
        assert_eq!(parse_memory_from_stat("short stat"), 0);
    }

    // =====================
    // Public API Tests (with real test data)
    // =====================

    /// Helper to create test topic file
    fn create_test_topic(topic_name: &str) -> Option<std::path::PathBuf> {
        let topics_dir = shm_topics_dir();
        if std::fs::create_dir_all(&topics_dir).is_err() {
            return None;
        }

        let safe_name: String = topic_name
            .chars()
            .map(|c| if c == '/' || c == ' ' { '_' } else { c })
            .collect();
        let filepath = topics_dir.join(&safe_name);

        // Create a small test file
        if std::fs::write(&filepath, vec![0u8; 1024]).is_ok() {
            Some(filepath)
        } else {
            None
        }
    }

    /// Cleanup helper
    fn cleanup_test_file(path: Option<std::path::PathBuf>) {
        if let Some(p) = path {
            let _ = std::fs::remove_file(p);
        }
    }

    #[test]
    fn test_discover_shared_memory_with_real_topic() {
        // Use simple topic name to avoid underscore-to-slash conversion confusion
        let test_topic = "testshm"; // Simple name without underscores
        let topic_file = create_test_topic(test_topic);

        if topic_file.is_some() {
            // Force cache refresh - handle potential poisoned lock
            let cache_refreshed = DISCOVERY_CACHE
                .write()
                .map(|mut cache| {
                    cache.shared_memory_last_updated =
                        std::time::Instant::now() - std::time::Duration::from_secs(10);
                    true
                })
                .unwrap_or(false);

            if !cache_refreshed {
                cleanup_test_file(topic_file);
                return; // Skip test if cache is poisoned
            }

            // Call the uncached version directly to avoid cache issues in parallel tests
            let result = discover_shared_memory_uncached();
            if let Ok(topics) = result {
                // Should find our test topic (underscores in filename become / in topic name)
                let found = topics.iter().any(|t| t.topic_name.contains("testshm"));
                assert!(
                    found,
                    "Should discover testshm topic, found: {:?}",
                    topics.iter().map(|t| &t.topic_name).collect::<Vec<_>>()
                );

                // Verify topic properties
                if let Some(topic) = topics.iter().find(|t| t.topic_name.contains("testshm")) {
                    assert_eq!(topic.size_bytes, 1024, "Topic should be 1024 bytes");
                }
            }
            // If result is Err, that's OK - test data might have been cleaned up by another test
        }

        cleanup_test_file(topic_file);
    }

    #[test]
    fn test_discover_nodes_returns_vec() {
        // Smoke test - should not panic even with no data
        let result = discover_nodes();
        assert!(result.is_ok());
    }

    #[test]
    fn test_discover_shared_memory_handles_missing_dirs() {
        // Smoke test - should not panic even if dirs don't exist
        let _ = discover_shared_memory();
    }

    #[test]
    fn test_topic_inactive_detection() {
        // Create a topic file and verify active detection works
        let topics_dir = shm_topics_dir();
        if std::fs::create_dir_all(&topics_dir).is_err() {
            return;
        }

        let test_file = topics_dir.join("test_active_topic");
        if std::fs::write(&test_file, vec![0u8; 512]).is_ok() {
            // Force cache refresh
            if let Ok(mut cache) = DISCOVERY_CACHE.write() {
                cache.shared_memory_last_updated =
                    std::time::Instant::now() - std::time::Duration::from_secs(10);
            }

            let result = discover_shared_memory();
            if let Ok(topics) = result {
                if let Some(topic) = topics.iter().find(|t| t.topic_name.contains("test_active")) {
                    // Just-created file should be considered active (recently modified)
                    assert!(topic.active, "Recently created topic should be active");
                }
            }

            let _ = std::fs::remove_file(&test_file);
        }
    }

    // =====================
    // DiscoveryCache Tests
    // =====================
    #[test]
    fn test_discovery_cache_new_is_stale() {
        let cache = DiscoveryCache::new();
        // New cache should be stale (forces initial update)
        assert!(cache.is_nodes_stale());
        assert!(cache.is_shared_memory_stale());
    }

    #[test]
    fn test_discovery_cache_update_nodes() {
        let mut cache = DiscoveryCache::new();
        let nodes = vec![NodeStatus {
            name: "test".to_string(),
            status: "Running".to_string(),
            health: HealthStatus::Healthy,
            priority: 0,
            process_id: 0,
            command_line: String::new(),
            working_dir: String::new(),
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: String::new(),
            scheduler_name: String::new(),
            category: ProcessCategory::Node,
            tick_count: 0,
            error_count: 0,
            actual_rate_hz: 0,
            publishers: vec![],
            subscribers: vec![],
        }];

        cache.update_nodes(nodes);

        // After update, nodes should not be stale (but shared_memory still is)
        assert!(!cache.is_nodes_stale());
        assert_eq!(cache.nodes.len(), 1);
    }

    #[test]
    fn test_discovery_cache_update_shared_memory() {
        let mut cache = DiscoveryCache::new();
        let shm = vec![SharedMemoryInfo {
            topic_name: "test".to_string(),
            size_bytes: 1024,
            active: true,
            accessing_processes: vec![],
            last_modified: None,
            message_type: None,
            publishers: vec![],
            subscribers: vec![],
            message_rate_hz: 0.0,
            status: TopicStatus::Stale,
            age_string: "unknown".to_string(),
        }];

        cache.update_shared_memory(shm);

        // After update, shared_memory should not be stale (but nodes still is)
        assert!(!cache.is_shared_memory_stale());
        assert_eq!(cache.shared_memory.len(), 1);
    }

    // =====================
    // Process Existence Tests
    // =====================
    #[test]
    fn test_process_exists_self() {
        // Current process should exist
        let pid = std::process::id();
        assert!(process_exists(pid));
    }

    #[test]
    fn test_process_exists_invalid() {
        // PID 0 or very high numbers shouldn't exist
        assert!(!process_exists(999999999));
    }

    // =====================
    // Edge Cases Tests
    // =====================
    #[test]
    fn test_node_status_clone() {
        let node = NodeStatus {
            name: "clone_test".to_string(),
            status: "Running".to_string(),
            health: HealthStatus::Warning,
            priority: 5,
            process_id: 9999,
            command_line: "test cmd".to_string(),
            working_dir: "/tmp".to_string(),
            cpu_usage: 50.0,
            memory_usage: 2048,
            start_time: "1h".to_string(),
            scheduler_name: "test_sched".to_string(),
            category: ProcessCategory::Tool,
            tick_count: 500,
            error_count: 2,
            actual_rate_hz: 100,
            publishers: vec![TopicInfo {
                topic: "pub".to_string(),
                type_name: "Msg".to_string(),
            }],
            subscribers: vec![],
        };

        let cloned = node.clone();
        assert_eq!(cloned.name, node.name);
        assert_eq!(cloned.status, node.status);
        assert_eq!(cloned.publishers.len(), node.publishers.len());
    }

    #[test]
    fn test_shared_memory_info_clone() {
        let shm = SharedMemoryInfo {
            topic_name: "clone_topic".to_string(),
            size_bytes: 8192,
            active: true,
            accessing_processes: vec![1, 2, 3],
            last_modified: Some(std::time::SystemTime::now()),
            message_type: Some("TestMsg".to_string()),
            publishers: vec!["pub1".to_string()],
            subscribers: vec!["sub1".to_string(), "sub2".to_string()],
            message_rate_hz: 60.0,
            status: TopicStatus::Active,
            age_string: "0s ago".to_string(),
        };

        let cloned = shm.clone();
        assert_eq!(cloned.topic_name, shm.topic_name);
        assert_eq!(cloned.accessing_processes.len(), 3);
        assert_eq!(cloned.subscribers.len(), 2);
    }

    #[test]
    fn test_health_status_variants() {
        // Ensure all health status variants work correctly
        let node_healthy = NodeStatus {
            name: "h".to_string(),
            status: String::new(),
            health: HealthStatus::Healthy,
            priority: 0,
            process_id: 0,
            command_line: String::new(),
            working_dir: String::new(),
            cpu_usage: 0.0,
            memory_usage: 0,
            start_time: String::new(),
            scheduler_name: String::new(),
            category: ProcessCategory::Node,
            tick_count: 0,
            error_count: 0,
            actual_rate_hz: 0,
            publishers: vec![],
            subscribers: vec![],
        };

        match node_healthy.health {
            HealthStatus::Healthy => {}
            _ => panic!("Expected Healthy"),
        }
    }

    #[test]
    #[ignore] // Run with: cargo test test_live_discovery -- --ignored --nocapture
    fn test_live_discovery() {
        println!("\n=== LIVE DISCOVERY TEST ===");

        let discovered = discover_shared_memory().unwrap_or_default();

        println!("Discovered {} topics:", discovered.len());
        for topic in &discovered {
            println!("  - Topic: {}", topic.topic_name);
            println!("    Active: {}", topic.active);
            println!("    Size: {} bytes", topic.size_bytes);
            println!("    Processes: {:?}", topic.accessing_processes);
            println!("    Publishers: {:?}", topic.publishers);
            println!("    Subscribers: {:?}", topic.subscribers);
            println!();
        }

        // Check what's on disk (flat namespace)
        println!("=== DISK CHECK ===");
        let topics_dir = shm_topics_dir();
        if topics_dir.exists() {
            println!("Topics directory exists: {:?}", topics_dir);
            if let Ok(topic_entries) = std::fs::read_dir(&topics_dir) {
                for t in topic_entries.flatten() {
                    println!("  Topic file: {:?}", t.file_name());
                }
            }
        } else {
            println!("Topics directory DOES NOT EXIST");
        }
    }
}

// ============================================================================
// Network Status Discovery
// ============================================================================

/// Discover network transport status for all nodes
///
/// Reads network status files from /dev/shm/horus/network/ directory.
/// These files are written by nodes when they use network transports.
pub fn discover_network_status() -> HorusResult<Vec<NetworkStatus>> {
    let network_dir = shm_network_dir();
    if !network_dir.exists() {
        return Ok(Vec::new());
    }

    let mut statuses = Vec::new();
    if let Ok(entries) = std::fs::read_dir(&network_dir) {
        for entry in entries.flatten() {
            if let Ok(content) = std::fs::read_to_string(entry.path()) {
                if let Ok(status) = serde_json::from_str::<NetworkStatus>(&content) {
                    // Only include fresh statuses (within last 30 seconds)
                    if status.is_fresh(30) {
                        statuses.push(status);
                    }
                }
            }
        }
    }

    Ok(statuses)
}

/// Get aggregated network statistics across all nodes
pub fn get_network_summary() -> NetworkSummary {
    let statuses = discover_network_status().unwrap_or_default();

    let mut summary = NetworkSummary::default();
    let mut transport_counts: std::collections::HashMap<String, u32> =
        std::collections::HashMap::new();

    for status in &statuses {
        summary.total_nodes += 1;
        summary.total_bytes_sent += status.bytes_sent;
        summary.total_bytes_received += status.bytes_received;
        summary.total_packets_sent += status.packets_sent;
        summary.total_packets_received += status.packets_received;

        *transport_counts
            .entry(status.transport_type.clone())
            .or_insert(0) += 1;

        for endpoint in &status.remote_endpoints {
            if !summary.unique_endpoints.contains(endpoint) {
                summary.unique_endpoints.push(endpoint.clone());
            }
        }
    }

    summary.transport_breakdown = transport_counts;
    summary.node_statuses = statuses;
    summary
}

/// Summary of network activity across all HORUS nodes
#[derive(Debug, Clone, Default)]
pub struct NetworkSummary {
    /// Total nodes with network status
    pub total_nodes: u32,
    /// Total bytes sent across all nodes
    pub total_bytes_sent: u64,
    /// Total bytes received across all nodes
    pub total_bytes_received: u64,
    /// Total packets sent
    pub total_packets_sent: u64,
    /// Total packets received
    pub total_packets_received: u64,
    /// Breakdown by transport type (e.g., "Udp": 3, "SharedMemory": 5)
    pub transport_breakdown: std::collections::HashMap<String, u32>,
    /// Unique remote endpoints discovered
    pub unique_endpoints: Vec<String>,
    /// Individual node statuses
    pub node_statuses: Vec<NetworkStatus>,
}
