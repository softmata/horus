//! Node command - Manage HORUS nodes
//!
//! Provides commands for listing, inspecting, and managing running nodes.

use crate::discovery::{discover_nodes, ProcessCategory};
use crate::progress::format_bytes;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::memory::platform::shm_control_dir;
use std::fs;
use std::process::Command;

/// List all running nodes
pub fn list_nodes(verbose: bool, json: bool, category: Option<String>) -> HorusResult<()> {
    let nodes = discover_nodes()?;

    // Filter by category if specified
    let filtered_nodes: Vec<_> = if let Some(ref cat) = category {
        let target_cat = match cat.to_lowercase().as_str() {
            "node" | "nodes" => ProcessCategory::Node,
            "tool" | "tools" => ProcessCategory::Tool,
            "cli" => ProcessCategory::CLI,
            _ => {
                return Err(HorusError::Config(format!(
                    "Unknown category '{}'. Valid options: node, tool, cli",
                    cat
                )));
            }
        };
        nodes
            .into_iter()
            .filter(|n| n.category == target_cat)
            .collect()
    } else {
        nodes
    };

    if json {
        let json_output = serde_json::to_string_pretty(
            &filtered_nodes
                .iter()
                .map(|n| {
                    serde_json::json!({
                        "name": n.name,
                        "status": n.status,
                        "health": format!("{:?}", n.health),
                        "priority": n.priority,
                        "pid": n.process_id,
                        "cpu_usage": n.cpu_usage,
                        "memory_usage": n.memory_usage,
                        "tick_count": n.tick_count,
                        "rate_hz": n.actual_rate_hz,
                        "error_count": n.error_count,
                        "category": format!("{:?}", n.category)
                    })
                })
                .collect::<Vec<_>>(),
        )
        .unwrap_or_default();
        println!("{}", json_output);
        return Ok(());
    }

    if filtered_nodes.is_empty() {
        println!("{}", "No running nodes found.".yellow());
        println!(
            "  {} Start a HORUS application to see nodes",
            "Tip:".dimmed()
        );
        return Ok(());
    }

    println!("{}", "Running Nodes:".green().bold());
    println!();

    if verbose {
        for node in &filtered_nodes {
            let status_color = match node.status.as_str() {
                "Running" | "running" => "Running".green(),
                "Idle" | "idle" => "Idle".yellow(),
                _ => node.status.as_str().red(),
            };

            println!("  {} {}", "Node:".cyan(), node.name.white().bold());
            println!("    {} {}", "Status:".dimmed(), status_color);
            println!("    {} {:?}", "Health:".dimmed(), node.health);
            println!("    {} {}", "Priority:".dimmed(), node.priority);
            println!("    {} {}", "PID:".dimmed(), node.process_id);
            println!("    {} {:.1}%", "CPU:".dimmed(), node.cpu_usage);
            println!(
                "    {} {} bytes",
                "Memory:".dimmed(),
                format_bytes(node.memory_usage)
            );
            println!("    {} {}", "Ticks:".dimmed(), node.tick_count);
            println!("    {} {} Hz", "Rate:".dimmed(), node.actual_rate_hz);
            if node.error_count > 0 {
                println!(
                    "    {} {}",
                    "Errors:".dimmed(),
                    node.error_count.to_string().red()
                );
            }
            if !node.publishers.is_empty() {
                println!(
                    "    {} {}",
                    "Publishes:".dimmed(),
                    node.publishers
                        .iter()
                        .map(|t| t.topic.clone())
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
            if !node.subscribers.is_empty() {
                println!(
                    "    {} {}",
                    "Subscribes:".dimmed(),
                    node.subscribers
                        .iter()
                        .map(|t| t.topic.clone())
                        .collect::<Vec<_>>()
                        .join(", ")
                );
            }
            println!();
        }
    } else {
        // Compact table view
        println!(
            "  {:<25} {:>8} {:>10} {:>8} {:>10} {:>8}",
            "NAME".dimmed(),
            "STATUS".dimmed(),
            "PRIORITY".dimmed(),
            "RATE".dimmed(),
            "TICKS".dimmed(),
            "PID".dimmed()
        );
        println!("  {}", "-".repeat(75).dimmed());

        for node in &filtered_nodes {
            let status = match node.status.as_str() {
                "Running" | "running" => "Running".green(),
                "Idle" | "idle" => "Idle".yellow(),
                _ => node.status.as_str().red(),
            };
            println!(
                "  {:<25} {:>8} {:>10} {:>6} Hz {:>10} {:>8}",
                truncate_name(&node.name, 25),
                status,
                node.priority,
                node.actual_rate_hz,
                node.tick_count,
                node.process_id
            );
        }
    }

    println!();
    println!("  {} {} node(s)", "Total:".dimmed(), filtered_nodes.len());

    Ok(())
}

/// Get detailed info about a specific node
pub fn node_info(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;

    let node = nodes.iter().find(|n| {
        n.name == name || n.name.ends_with(&format!("/{}", name)) || n.name.contains(name)
    });

    if node.is_none() {
        return Err(HorusError::Config(format!(
            "Node '{}' not found. Use 'horus node list' to see running nodes.",
            name
        )));
    }

    let node = node.unwrap();

    println!("{}", "Node Information".green().bold());
    println!();
    println!("  {} {}", "Name:".cyan(), node.name.white().bold());

    let status_color = match node.status.as_str() {
        "Running" | "running" => "Running".green(),
        "Idle" | "idle" => "Idle".yellow(),
        _ => node.status.as_str().red(),
    };
    println!("  {} {}", "Status:".cyan(), status_color);
    println!("  {} {:?}", "Health:".cyan(), node.health);
    println!("  {} {:?}", "Category:".cyan(), node.category);

    println!();
    println!("  {}", "Process Info:".cyan());
    println!("    {} {}", "PID:".dimmed(), node.process_id);
    println!("    {} {}", "Priority:".dimmed(), node.priority);
    if !node.scheduler_name.is_empty() {
        println!("    {} {}", "Scheduler:".dimmed(), node.scheduler_name);
    }
    if !node.working_dir.is_empty() {
        println!("    {} {}", "Working Dir:".dimmed(), node.working_dir);
    }

    println!();
    println!("  {}", "Performance:".cyan());
    println!("    {} {:.1}%", "CPU Usage:".dimmed(), node.cpu_usage);
    println!(
        "    {} {}",
        "Memory:".dimmed(),
        format_bytes(node.memory_usage)
    );
    println!("    {} {} Hz", "Tick Rate:".dimmed(), node.actual_rate_hz);
    println!("    {} {}", "Total Ticks:".dimmed(), node.tick_count);
    println!(
        "    {} {}",
        "Errors:".dimmed(),
        if node.error_count == 0 {
            "0".green()
        } else {
            node.error_count.to_string().red()
        }
    );

    println!();
    println!("  {}", "Publications:".cyan());
    if node.publishers.is_empty() {
        println!("    {}", "(none)".dimmed());
    } else {
        for pub_topic in &node.publishers {
            let msg_type = if pub_topic.type_name.is_empty() {
                "unknown"
            } else {
                &pub_topic.type_name
            };
            println!("    - {} ({})", pub_topic.topic, msg_type.dimmed());
        }
    }

    println!();
    println!("  {}", "Subscriptions:".cyan());
    if node.subscribers.is_empty() {
        println!("    {}", "(none)".dimmed());
    } else {
        for sub_topic in &node.subscribers {
            let msg_type = if sub_topic.type_name.is_empty() {
                "unknown"
            } else {
                &sub_topic.type_name
            };
            println!("    - {} ({})", sub_topic.topic, msg_type.dimmed());
        }
    }

    Ok(())
}

/// Kill a running node (via IPC control file - only stops the specific node)
pub fn kill_node(name: &str, force: bool) -> HorusResult<()> {
    let nodes = discover_nodes()?;

    let node = nodes.iter().find(|n| {
        n.name == name || n.name.ends_with(&format!("/{}", name)) || n.name.contains(name)
    });

    if node.is_none() {
        return Err(HorusError::Config(format!(
            "Node '{}' not found. Use 'horus node list' to see running nodes.",
            name
        )));
    }

    let node = node.unwrap();

    println!("{} Stopping node: {}", "".cyan(), node.name.white().bold());

    // Write control file to stop the specific node
    let control_dir = shm_control_dir();
    fs::create_dir_all(&control_dir)
        .map_err(|e| HorusError::Config(format!("Failed to create control directory: {}", e)))?;

    let control_file = control_dir.join(format!("{}.cmd", node.name));
    fs::write(&control_file, "stop")
        .map_err(|e| HorusError::Config(format!("Failed to write control file: {}", e)))?;

    println!("{} Node stop command sent", "".green());
    println!(
        "  {} The scheduler will stop this node on next tick",
        "Note:".dimmed()
    );

    // If force flag is set, also kill the process (fallback for stuck nodes)
    if force {
        let pid = node.process_id;
        if pid != 0 {
            println!("{} Force killing process (PID: {})", "".yellow(), pid);
            let _ = Command::new("kill").arg("-9").arg(pid.to_string()).output();
        }
    }

    Ok(())
}

/// Restart a node (via IPC control file - re-initializes the specific node)
pub fn restart_node(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;

    let node = nodes.iter().find(|n| {
        n.name == name || n.name.ends_with(&format!("/{}", name)) || n.name.contains(name)
    });

    if node.is_none() {
        return Err(HorusError::Config(format!(
            "Node '{}' not found. Use 'horus node list' to see running nodes.",
            name
        )));
    }

    let node = node.unwrap();

    println!(
        "{} Restarting node: {}",
        "".cyan(),
        node.name.white().bold()
    );

    // Write control file to restart the specific node
    let control_dir = shm_control_dir();
    fs::create_dir_all(&control_dir)
        .map_err(|e| HorusError::Config(format!("Failed to create control directory: {}", e)))?;

    let control_file = control_dir.join(format!("{}.cmd", node.name));
    fs::write(&control_file, "restart")
        .map_err(|e| HorusError::Config(format!("Failed to write control file: {}", e)))?;

    println!("{} Node restart command sent", "".green());
    println!(
        "  {} The scheduler will re-initialize this node on next tick",
        "Note:".dimmed()
    );

    Ok(())
}

/// Pause a running node (via IPC control file)
pub fn pause_node(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;

    let node = nodes.iter().find(|n| {
        n.name == name || n.name.ends_with(&format!("/{}", name)) || n.name.contains(name)
    });

    if node.is_none() {
        return Err(HorusError::Config(format!(
            "Node '{}' not found. Use 'horus node list' to see running nodes.",
            name
        )));
    }

    let node = node.unwrap();

    println!("{} Pausing node: {}", "".cyan(), node.name.white().bold());

    // Write control file to pause the specific node
    let control_dir = shm_control_dir();
    fs::create_dir_all(&control_dir)
        .map_err(|e| HorusError::Config(format!("Failed to create control directory: {}", e)))?;

    let control_file = control_dir.join(format!("{}.cmd", node.name));
    fs::write(&control_file, "pause")
        .map_err(|e| HorusError::Config(format!("Failed to write control file: {}", e)))?;

    println!("{} Node paused", "".green());
    println!(
        "  {} Use 'horus node resume {}' to resume execution",
        "Tip:".dimmed(),
        name
    );

    Ok(())
}

/// Resume a paused node (via IPC control file)
pub fn resume_node(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;

    let node = nodes.iter().find(|n| {
        n.name == name || n.name.ends_with(&format!("/{}", name)) || n.name.contains(name)
    });

    if node.is_none() {
        return Err(HorusError::Config(format!(
            "Node '{}' not found. Use 'horus node list' to see running nodes.",
            name
        )));
    }

    let node = node.unwrap();

    println!("{} Resuming node: {}", "".cyan(), node.name.white().bold());

    // Write control file to resume the specific node
    let control_dir = shm_control_dir();
    fs::create_dir_all(&control_dir)
        .map_err(|e| HorusError::Config(format!("Failed to create control directory: {}", e)))?;

    let control_file = control_dir.join(format!("{}.cmd", node.name));
    fs::write(&control_file, "resume")
        .map_err(|e| HorusError::Config(format!("Failed to write control file: {}", e)))?;

    println!("{} Node resumed", "".green());

    Ok(())
}

/// Truncate name to fit in column
fn truncate_name(name: &str, max_len: usize) -> String {
    if name.len() <= max_len {
        name.to_string()
    } else {
        format!("{}...", &name[..max_len - 3])
    }
}
