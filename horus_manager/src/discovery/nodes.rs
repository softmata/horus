use super::*;

pub(super) fn discover_nodes_uncached() -> HorusResult<Vec<NodeStatus>> {
    use horus_core::scheduling::registry::SchedulerRegistry;

    // PRIMARY: Delegate to horus_sys cross-platform discovery
    let sys_nodes = horus_sys::discover::find_nodes();
    let mut nodes: Vec<NodeStatus> = sys_nodes
        .into_iter()
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

    Ok(nodes)
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
