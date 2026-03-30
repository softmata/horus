use super::*;

pub(super) fn discover_nodes_uncached() -> HorusResult<Vec<NodeStatus>> {
    use horus_core::scheduling::SchedulerRegistry;

    // PRIMARY: Delegate to horus_sys cross-platform discovery
    let sys_nodes = horus_sys::discover::find_nodes();

    // Auto-cleanup stale LOCAL node presence files from dead processes.
    // find_nodes() already skips dead PIDs, so stale files never appear in results.
    // We scan the directory directly to find and remove orphaned .json files.
    // Skip remote_* and bridged_* files — those are managed by horus_net.
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    if let Ok(entries) = std::fs::read_dir(&nodes_dir) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.extension().is_none_or(|e| e != "json") {
                continue;
            }
            let fname = path.file_name().and_then(|n| n.to_str()).unwrap_or("");
            if fname.starts_with("remote_") || fname.starts_with("bridged_") {
                continue;
            }
            if let Ok(content) = std::fs::read_to_string(&path) {
                if let Some(pid_str) = content
                    .split("\"pid\":")
                    .nth(1)
                    .and_then(|s| s.split(&[',', '}'][..]).next())
                {
                    if let Ok(pid) = pid_str.trim().parse::<i32>() {
                        if pid > 0 && unsafe { libc::kill(pid, 0) != 0 } {
                            let _ = std::fs::remove_file(&path);
                        }
                    }
                }
            }
        }
    }

    let mut nodes: Vec<NodeStatus> = sys_nodes
        .into_iter()
        .filter(|n| {
            // Filter out dead nodes — match ROS2 behavior (dead nodes simply vanish)
            if n.pid > 0 {
                unsafe { libc::kill(n.pid as i32, 0) == 0 }
            } else {
                false
            }
        })
        .map(|n| {
            let publishers = n
                .publishers
                .iter()
                .map(|t| TopicInfo {
                    topic: t.topic.clone(),
                    type_name: t.type_name.clone(),
                })
                .collect();
            let subscribers = n
                .subscribers
                .iter()
                .map(|t| TopicInfo {
                    topic: t.topic.clone(),
                    type_name: t.type_name.clone(),
                })
                .collect();
            NodeStatus {
                name: n.name,
                status: "Running".to_string(),
                health: match n.health.as_str() {
                    "Warning" => HealthStatus::Warning,
                    "Error" => HealthStatus::Error,
                    "Critical" => HealthStatus::Critical,
                    _ => HealthStatus::Healthy,
                },
                priority: n.priority,
                process_id: n.pid,
                command_line: n.cmdline,
                working_dir: n.working_dir,
                cpu_usage: n.cpu_percent,
                memory_usage: n.memory_kb,
                start_time: n.start_time,
                scheduler_name: n.scheduler.unwrap_or_default(),
                category: ProcessCategory::Node,
                tick_count: n.tick_count,
                error_count: n.error_count,
                actual_rate_hz: n.rate_hz.map(|r| r as u32).unwrap_or(0),
                publishers,
                subscribers,
                live_tick_count: None,
                live_health: None,
                live_avg_tick_ns: None,
                live_max_tick_ns: None,
                live_budget_misses: None,
                live_deadline_misses: None,
                live_watchdog_severity: None,
                live_p99_tick_ns: None,
                is_remote: false,
                host_id: String::new(),
                is_bridged: false,
                bridge_protocol: String::new(),
                launch_session: None,
                launch_name: None,
            }
        })
        .collect();

    // MERGE: Read live metrics from SHM SchedulerRegistry files
    let registries = SchedulerRegistry::read_all_registries();
    for (_scheduler_name, slots) in &registries {
        for slot in slots {
            // Find the matching node by name and merge live data
            if let Some(node) = nodes.iter_mut().find(|n| n.name == slot.name) {
                node.live_tick_count = Some(slot.tick_count);
                node.live_health = Some(slot.health);
                node.live_avg_tick_ns = Some(slot.avg_tick_ns);
                node.live_max_tick_ns = Some(slot.max_tick_ns);
                node.live_budget_misses = Some(slot.budget_misses);
                node.live_deadline_misses = Some(slot.deadline_misses);
                node.live_watchdog_severity = Some(slot.watchdog_severity);
                node.live_p99_tick_ns = Some(slot.p99_tick_ns);
                // Override stale presence health with live health
                node.health = match slot.health {
                    1 => HealthStatus::Warning,
                    2 => HealthStatus::Error,
                    3 => HealthStatus::Critical,
                    _ => HealthStatus::Healthy,
                };
                // Override stale presence tick_count with live count
                node.tick_count = slot.tick_count;
                node.error_count = slot.error_count;
            }
        }
    }

    // EXTRA: Add any other HORUS processes (tools, CLIs) not detected via presence
    if let Ok(process_nodes) = discover_horus_processes() {
        for process_node in process_nodes {
            if !nodes
                .iter()
                .any(|n| n.process_id == process_node.process_id || n.name == process_node.name)
            {
                nodes.push(process_node);
            }
        }
    }

    // REMOTE: Read remote presence files (from horus_net and bridge plugins)
    let nodes_dir = horus_sys::shm::shm_nodes_dir();
    if let Ok(entries) = std::fs::read_dir(&nodes_dir) {
        for entry in entries.flatten() {
            let fname = entry.file_name();
            let fname_str = fname.to_string_lossy();

            // remote_{host_id}.json — horus_net presence
            // bridged_{proto}_{node}.json — bridge plugin presence
            let is_remote_file = fname_str.starts_with("remote_") && fname_str.ends_with(".json");
            let is_bridged_file = fname_str.starts_with("bridged_") && fname_str.ends_with(".json");

            if !is_remote_file && !is_bridged_file {
                continue;
            }

            let content = match std::fs::read_to_string(entry.path()) {
                Ok(c) => c,
                Err(_) => continue,
            };

            // Parse JSON (minimal — just extract host_id and node names)
            // Using string search to avoid serde dependency in horus_manager discovery
            let host_id = extract_json_string(&content, "host_id").unwrap_or_default();
            let is_bridged = extract_json_bool(&content, "is_bridged").unwrap_or(false);
            let bridge_proto = extract_json_string(&content, "bridge_protocol").unwrap_or_default();
            let last_seen_ns = extract_json_u64(&content, "last_seen_ns").unwrap_or(0);

            // Staleness check (heartbeat-based, not PID)
            let now_ns = std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap_or_default()
                .as_nanos() as u64;
            let age_secs = (now_ns.saturating_sub(last_seen_ns)) / 1_000_000_000;
            let status = if age_secs > 30 {
                // Dead — skip (cleanup handled by PresenceReceiver)
                continue;
            } else if age_secs > 3 {
                "Unreachable".to_string()
            } else {
                "Running".to_string()
            };

            // Extract node names from JSON "nodes" array
            // Handles both "name":"value" and "name": "value" (with optional whitespace)
            if let Some(nodes_start) = content.find("\"nodes\"") {
                let nodes_section = &content[nodes_start..];
                let mut search_pos = 0;
                while let Some(name_key_pos) = nodes_section[search_pos..].find("\"name\"") {
                    // Skip past "name" + optional whitespace + : + optional whitespace + "
                    let after_key = search_pos + name_key_pos + 6; // skip "name"
                    let rest = &nodes_section[after_key..];
                    let rest = rest.trim_start();
                    if !rest.starts_with(':') {
                        search_pos = after_key;
                        continue;
                    }
                    let rest = rest[1..].trim_start(); // skip : and whitespace
                    if !rest.starts_with('"') {
                        search_pos = after_key;
                        continue;
                    }
                    let value_start = nodes_section.len() - rest.len() + 1; // +1 to skip opening "
                    let start = value_start;
                    if let Some(end) = nodes_section[start..].find('"') {
                        let node_name = &nodes_section[start..start + end];

                        nodes.push(NodeStatus {
                            name: node_name.to_string(),
                            status: status.clone(),
                            health: if status == "Unreachable" {
                                HealthStatus::Warning
                            } else {
                                HealthStatus::Healthy
                            },
                            priority: 0,
                            process_id: 0, // No PID for remote nodes
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
                            publishers: Vec::new(),
                            subscribers: Vec::new(),
                            live_tick_count: None,
                            live_health: None,
                            live_avg_tick_ns: None,
                            live_max_tick_ns: None,
                            live_budget_misses: None,
                            live_deadline_misses: None,
                            live_watchdog_severity: None,
                            live_p99_tick_ns: None,
                            is_remote: !is_bridged,
                            host_id: host_id.clone(),
                            is_bridged,
                            bridge_protocol: bridge_proto.clone(),
                            launch_session: None,
                            launch_name: None,
                        });

                        search_pos = start + end + 1;
                    } else {
                        break;
                    }
                }
            }
        }
    }

    // LAUNCH SESSION: Annotate nodes with launch session membership
    let sessions = crate::commands::launch::LaunchSession::read_all();
    for session in &sessions {
        for entry in &session.processes {
            for node in nodes.iter_mut() {
                if node.process_id == entry.pid || entry.node_names.iter().any(|n| n == &node.name)
                {
                    node.launch_session = Some(session.session.clone());
                    node.launch_name = Some(entry.name.clone());
                }
            }
        }
    }

    Ok(nodes)
}

// ── Minimal JSON helpers (avoid serde dependency) ────────────────────────────

fn extract_json_string(json: &str, key: &str) -> Option<String> {
    let pattern = format!("\"{}\":\"", key);
    let start = json.find(&pattern)? + pattern.len();
    let end = json[start..].find('"')?;
    Some(json[start..start + end].to_string())
}

fn extract_json_bool(json: &str, key: &str) -> Option<bool> {
    let pattern = format!("\"{}\":", key);
    let start = json.find(&pattern)? + pattern.len();
    let rest = json[start..].trim_start();
    if rest.starts_with("true") {
        Some(true)
    } else if rest.starts_with("false") {
        Some(false)
    } else {
        None
    }
}

fn extract_json_u64(json: &str, key: &str) -> Option<u64> {
    let pattern = format!("\"{}\":", key);
    let start = json.find(&pattern)? + pattern.len();
    let rest = json[start..].trim_start();
    let end = rest
        .find(|c: char| !c.is_ascii_digit())
        .unwrap_or(rest.len());
    rest[..end].parse().ok()
}

fn discover_horus_processes() -> anyhow::Result<Vec<NodeStatus>> {
    let mut nodes = Vec::new();

    for pid in horus_sys::discover::list_pids() {
        // Fast skip: kernel threads and very low PIDs
        if pid < 100 {
            continue;
        }

        let proc_info = match horus_sys::discover::process_info(pid) {
            Ok(info) if !info.cmdline.is_empty() => info,
            _ => continue,
        };

        if should_track_process(&proc_info.cmdline) {
            let name = extract_process_name(&proc_info.cmdline);
            let category = categorize_process(&name, &proc_info.cmdline);

            nodes.push(NodeStatus {
                name,
                status: "Running".to_string(),
                health: HealthStatus::Unknown,
                priority: 0,
                process_id: pid,
                command_line: proc_info.cmdline,
                working_dir: proc_info.working_dir,
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
                live_tick_count: None,
                live_health: None,
                live_avg_tick_ns: None,
                live_max_tick_ns: None,
                live_budget_misses: None,
                live_deadline_misses: None,
                live_watchdog_severity: None,
                live_p99_tick_ns: None,
                is_remote: false,
                host_id: String::new(),
                is_bridged: false,
                bridge_protocol: String::new(),
                launch_session: None,
                launch_name: None,
            });
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

#[allow(dead_code)] // used by tests
pub(crate) fn process_exists(pid: u32) -> bool {
    horus_sys::process::ProcessHandle::from_pid(pid).is_alive()
}

// ── Pure text-parsing utilities (test-only, no platform deps) ────────────

/// Parse fields from `/proc/[pid]/stat` safely, handling process names
/// that contain spaces or parentheses.
#[cfg(test)]
pub(super) fn parse_stat_fields(stat_content: &str) -> Option<Vec<&str>> {
    let after_comm = stat_content.rfind(')')?.checked_add(1)?;
    if after_comm >= stat_content.len() {
        return None;
    }
    let fields: Vec<&str> = stat_content[after_comm..].split_whitespace().collect();
    if fields.is_empty() {
        return None;
    }
    Some(fields)
}

#[cfg(test)]
pub(crate) fn parse_memory_from_stat(stat: &str) -> u64 {
    let fields = match parse_stat_fields(stat) {
        Some(f) if f.len() > 21 => f,
        _ => return 0,
    };
    // field[21] = rss (in pages), assume 4KB pages for cross-platform parsing
    fields[21].parse::<u64>().unwrap_or(0) * 4
}

#[cfg(test)]
pub(crate) fn format_duration(duration: std::time::Duration) -> String {
    horus_sys::discover::format_duration(duration)
}
