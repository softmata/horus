use super::*;

pub(super) fn discover_nodes_uncached() -> HorusResult<Vec<NodeStatus>> {
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

/// Discover all scheduler registry files in home directory (cross-platform)
pub(crate) fn discover_registry_files() -> Vec<std::path::PathBuf> {
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

pub(crate) fn should_track_process(cmdline: &str) -> bool {
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

pub(crate) fn categorize_process(name: &str, cmdline: &str) -> ProcessCategory {
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

pub(crate) fn extract_process_name(cmdline: &str) -> String {
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

pub(crate) fn process_exists(pid: u32) -> bool {
    #[cfg(target_os = "linux")]
    {
        Path::new(&format!("/proc/{}", pid)).exists()
    }
    #[cfg(target_os = "macos")]
    {
        // macOS: use kill(0) to check if process exists
        // SAFETY: pid is a valid process ID from /proc or system API; signal 0 only checks existence.
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
        // SAFETY: OpenProcess returns a valid handle or null; CloseHandle is called on success.
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
        // SAFETY: pid is a valid process ID; signal 0 only checks existence without sending a signal.
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
            _pid: pid,
            _name: name,
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
            _pid: pid,
            _name: format!("pid_{}", pid),
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
pub(crate) fn parse_memory_from_stat(stat: &str) -> u64 {
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

pub(crate) fn format_duration(duration: std::time::Duration) -> String {
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
        _pid: pid,
        _name: name,
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

    // SAFETY: buf is a properly sized buffer; pid is a valid process ID from the caller.
    let result = unsafe { proc_name(pid as i32, buf.as_mut_ptr(), buf.len() as u32) };

    if result > 0 {
        // SAFETY: proc_name wrote a null-terminated string into buf (result > 0 confirms success).
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
    // SAFETY: mib is a valid 3-element array for KERN_PROCARGS2; null output buffer queries size only.
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
    // SAFETY: buf is allocated to the size returned by the first sysctl call; mib unchanged.
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
    let argc_raw = i32::from_ne_bytes([buf[0], buf[1], buf[2], buf[3]]);
    if argc_raw < 0 {
        return None;
    }
    let argc = argc_raw as usize;
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

    // SAFETY: VnodePathInfo is a C-compatible repr(C) struct where all-zeros is a valid representation.
    let mut info: VnodePathInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<VnodePathInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized for PROC_PIDVNODEPATHINFO flavor.
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

    // SAFETY: TaskInfo is a repr(C) struct where all-zeros is a valid representation.
    let mut info: TaskInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<TaskInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized for PROC_PIDTASKINFO flavor.
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

    // SAFETY: TaskInfo is a repr(C) struct where all-zeros is a valid representation.
    let mut info: TaskInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<TaskInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized for PROC_PIDTASKINFO flavor.
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

    // SAFETY: BsdInfo is a repr(C) struct where all-zeros is a valid representation.
    let mut info: BsdInfo = unsafe { std::mem::zeroed() };
    let size = std::mem::size_of::<BsdInfo>() as i32;

    // SAFETY: pid is a valid process ID; buffer is properly sized for PROC_PIDTBSDINFO flavor.
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
    // SAFETY: pid is a valid process ID; requesting query-only access flags.
    let handle = unsafe { OpenProcess(PROCESS_QUERY_INFORMATION | PROCESS_VM_READ, 0, pid) };

    if handle.is_null() {
        return Ok(ProcessInfo {
            _pid: pid,
            _name: format!("pid_{}", pid),
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

    // SAFETY: handle is a valid process handle obtained from OpenProcess above.
    unsafe { CloseHandle(handle) };

    Ok(ProcessInfo {
        _pid: pid,
        _name: name,
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
    // SAFETY: handle is a valid process handle with PROCESS_QUERY_INFORMATION access; buf is MAX_PATH-sized.
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

    // SAFETY: pid is a valid process ID; requesting limited query access.
    let handle = unsafe { OpenProcess(PROCESS_QUERY_LIMITED_INFORMATION, 0, pid) };
    if handle.is_null() {
        return None;
    }

    let mut buf = vec![0u16; MAX_PATH];
    let mut size = MAX_PATH as u32;

    // SAFETY: handle is valid from OpenProcess; buf is MAX_PATH-sized; size is in/out parameter.
    let result = unsafe { QueryFullProcessImageNameW(handle, 0, buf.as_mut_ptr(), &mut size) };

    // SAFETY: handle is a valid process handle obtained from OpenProcess above.
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

    // SAFETY: ProcessMemoryCounters is a repr(C) struct where all-zeros is a valid representation.
    let mut counters: ProcessMemoryCounters = unsafe { std::mem::zeroed() };
    counters.cb = std::mem::size_of::<ProcessMemoryCounters>() as u32;

    // SAFETY: handle is a valid process handle; counters.cb is set to the correct struct size.
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

    // SAFETY: handle is a valid process handle; all Filetime output pointers are properly initialized.
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

    // SAFETY: handle is a valid process handle; all Filetime output pointers are properly initialized.
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
