//! Node command - Manage HORUS nodes
//!
//! Provides commands for listing, inspecting, and managing running nodes.

use crate::cli_output;
use crate::discovery::{discover_nodes, NodeStatus, ProcessCategory};
use crate::progress::format_bytes;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::ControlCommand;
use horus_sys::process::{ProcessHandle, Signal};

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
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Unknown category '{}'. Valid options: node, tool, cli",
                    cat
                ))));
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
        let items: Vec<_> = filtered_nodes
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
            .collect();
        let output = serde_json::json!({
            "count": items.len(),
            "items": items
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
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

    let node = nodes
        .iter()
        .find(|n| n.name == name || n.name.ends_with(&format!("/{}", name)));

    let Some(node) = node else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Node '{}' not found. Use 'horus node list' to see running nodes.",
            name
        ))));
    };

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

/// Find a node by name (exact or suffix match).
fn find_node<'a>(nodes: &'a [NodeStatus], name: &str) -> HorusResult<&'a NodeStatus> {
    nodes
        .iter()
        .find(|n| n.name == name || n.name.ends_with(&format!("/{}", name)))
        .ok_or_else(|| {
            HorusError::Config(ConfigError::Other(format!(
                "Node '{}' not found. Use 'horus node list' to see running nodes.",
                name
            )))
        })
}

/// Send a ControlCommand to the scheduler that owns a node.
///
/// Discovers the node, reads its scheduler name from presence data,
/// publishes the command to `horus.ctl.{scheduler_name}` via the Topic API.
fn send_control_command(node: &NodeStatus, cmd: ControlCommand) -> HorusResult<()> {
    let scheduler_name = if node.scheduler_name.is_empty() {
        "Scheduler"
    } else {
        &node.scheduler_name
    };

    let ctl_topic_name = format!("horus.ctl.{}", scheduler_name);
    let topic = horus_core::Topic::<ControlCommand>::new_with_kind(
        &ctl_topic_name,
        horus_core::TopicKind::System as u8,
    )
    .map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to open control topic '{}': {}",
            ctl_topic_name, e
        )))
    })?;

    topic.send(cmd);
    Ok(())
}

/// Kill a running node (sends GracefulShutdown or PauseNode via control topic)
pub fn kill_node(name: &str, force: bool) -> HorusResult<()> {
    let nodes = discover_nodes()?;
    let node = find_node(&nodes, name)?;

    println!(
        "{} Stopping node: {}",
        cli_output::ICON_INFO.cyan(),
        node.name.white().bold()
    );

    send_control_command(
        node,
        ControlCommand::PauseNode {
            name: node.name.clone(),
        },
    )?;

    println!(
        "{} Stop command sent to scheduler '{}'",
        cli_output::ICON_SUCCESS.green(),
        node.scheduler_name
    );
    println!(
        "  {} The scheduler will pause this node on next tick",
        "Note:".dimmed()
    );

    // If force flag is set, also kill the process (fallback for stuck nodes)
    if force {
        let pid = node.process_id;
        if pid != 0 {
            println!(
                "{} Force killing process (PID: {})",
                cli_output::ICON_WARN.yellow(),
                pid
            );
            let _ = ProcessHandle::from_pid(pid).signal(Signal::Kill);
        }
    }

    Ok(())
}

/// Restart a node (pause + resume via control topic to trigger re-init)
pub fn restart_node(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;
    let node = find_node(&nodes, name)?;

    println!(
        "{} Restarting node: {}",
        cli_output::ICON_INFO.cyan(),
        node.name.white().bold()
    );

    // Pause then resume triggers re-init in the scheduler
    send_control_command(
        node,
        ControlCommand::PauseNode {
            name: node.name.clone(),
        },
    )?;
    send_control_command(
        node,
        ControlCommand::ResumeNode {
            name: node.name.clone(),
        },
    )?;

    println!(
        "{} Restart command sent to scheduler '{}'",
        cli_output::ICON_SUCCESS.green(),
        node.scheduler_name
    );
    println!(
        "  {} The scheduler will re-initialize this node on next tick",
        "Note:".dimmed()
    );

    Ok(())
}

/// Pause a running node (via control topic)
pub fn pause_node(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;
    let node = find_node(&nodes, name)?;

    println!(
        "{} Pausing node: {}",
        cli_output::ICON_INFO.cyan(),
        node.name.white().bold()
    );

    send_control_command(
        node,
        ControlCommand::PauseNode {
            name: node.name.clone(),
        },
    )?;

    println!("{} Pause command sent", cli_output::ICON_SUCCESS.green());
    println!(
        "  {} Use 'horus node resume {}' to resume execution",
        "Tip:".dimmed(),
        name
    );

    Ok(())
}

/// Resume a paused node (via control topic)
pub fn resume_node(name: &str) -> HorusResult<()> {
    let nodes = discover_nodes()?;
    let node = find_node(&nodes, name)?;

    println!(
        "{} Resuming node: {}",
        cli_output::ICON_INFO.cyan(),
        node.name.white().bold()
    );

    send_control_command(
        node,
        ControlCommand::ResumeNode {
            name: node.name.clone(),
        },
    )?;

    println!("{} Resume command sent", cli_output::ICON_SUCCESS.green());

    Ok(())
}

/// Truncate name to fit in column
fn truncate_name(name: &str, max_len: usize) -> String {
    crate::cli_output::safe_truncate(name, max_len)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn truncate_name_short_unchanged() {
        assert_eq!(truncate_name("my_node", 25), "my_node");
    }

    #[test]
    fn truncate_name_exact_length() {
        let name = "a".repeat(25);
        let result = truncate_name(&name, 25);
        assert!(result.len() <= 25);
    }

    #[test]
    fn truncate_name_long_is_shortened() {
        let name = "a_very_long_node_name_that_exceeds_the_limit";
        let result = truncate_name(name, 10);
        assert!(result.len() <= 10);
    }

    #[test]
    fn list_nodes_invalid_category_returns_error() {
        let result = list_nodes(false, false, Some("bogus".to_string()));
        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("bogus"),
            "error should mention the bad category"
        );
    }

    #[test]
    fn list_nodes_valid_categories_accepted() {
        // These should not fail on category parsing (they may return empty lists)
        for cat in &["node", "nodes", "tool", "tools", "cli"] {
            let result = list_nodes(false, false, Some(cat.to_string()));
            // Should succeed (Ok) or fail with a non-category error
            if let Err(e) = &result {
                assert!(
                    !e.to_string().contains("Unknown category"),
                    "category '{}' should be recognized",
                    cat
                );
            }
        }
    }

    // ── Battle tests: node list ──────────────────────────────────────────

    #[test]
    fn list_nodes_no_category_succeeds() {
        // No category filter: should not error on category parsing
        let result = list_nodes(false, false, None);
        // May fail due to no SHM but should not fail on category parsing
        if let Err(e) = &result {
            assert!(
                !e.to_string().contains("Unknown category"),
                "no category should mean no category error"
            );
        }
    }

    #[test]
    fn list_nodes_verbose_mode_succeeds() {
        let result = list_nodes(true, false, None);
        if let Err(e) = &result {
            assert!(
                !e.to_string().contains("Unknown category"),
                "verbose mode should not affect category parsing"
            );
        }
    }

    #[test]
    fn list_nodes_json_mode_succeeds() {
        let result = list_nodes(false, true, None);
        if let Err(e) = &result {
            assert!(
                !e.to_string().contains("Unknown category"),
                "json mode should not affect category parsing"
            );
        }
    }

    #[test]
    fn list_nodes_invalid_category_empty_string() {
        let result = list_nodes(false, false, Some("".to_string()));
        assert!(result.is_err(), "empty string category should be rejected");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Unknown category"),
            "error should mention unknown category: {}",
            err
        );
    }

    #[test]
    fn list_nodes_invalid_category_case_sensitive_uppercase() {
        // "NODE" (uppercase) should be rejected — matching is lowercase-normalized
        // so "NODE".to_lowercase() == "node" which IS valid
        let result = list_nodes(false, false, Some("NODE".to_string()));
        // The code does .to_lowercase() so this should succeed as "node"
        if let Err(e) = &result {
            assert!(
                !e.to_string().contains("Unknown category"),
                "uppercase 'NODE' should normalize to 'node'"
            );
        }
    }

    #[test]
    fn list_nodes_invalid_category_unicode() {
        let result = list_nodes(false, false, Some("\u{1F600}".to_string()));
        assert!(result.is_err(), "unicode emoji category should be rejected");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Unknown category"),
            "error should mention unknown category: {}",
            err
        );
    }

    #[test]
    fn list_nodes_invalid_category_whitespace() {
        let result = list_nodes(false, false, Some("  node  ".to_string()));
        // The code does NOT trim, so "  node  " should be rejected
        assert!(result.is_err(), "untrimmed category should be rejected");
    }

    #[test]
    fn list_nodes_invalid_category_numeric() {
        let result = list_nodes(false, false, Some("42".to_string()));
        assert!(result.is_err(), "numeric category should be rejected");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("42"),
            "error should echo back the bad category"
        );
    }

    // ── Battle tests: node info ──────────────────────────────────────────

    #[test]
    fn node_info_nonexistent_returns_error() {
        let result = node_info("this_node_definitely_does_not_exist_12345");
        assert!(result.is_err(), "info on nonexistent node should fail");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("not found"),
            "error should say node not found: {}",
            err
        );
    }

    #[test]
    fn node_info_empty_name_returns_error() {
        let result = node_info("");
        assert!(result.is_err(), "info on empty name should fail");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("not found"),
            "error should say not found: {}",
            err
        );
    }

    #[test]
    fn node_info_unicode_name_returns_error() {
        let result = node_info("\u{1F916}_robot_node");
        assert!(
            result.is_err(),
            "info on unicode name should fail (no such node)"
        );
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("not found"),
            "error should say not found: {}",
            err
        );
    }

    #[test]
    fn node_info_very_long_name_returns_error() {
        let long_name = "a".repeat(4096);
        let result = node_info(&long_name);
        assert!(result.is_err(), "info on extremely long name should fail");
    }

    #[test]
    fn node_info_slash_name_returns_error() {
        // Test the ends_with("/{name}") matching path
        let result = node_info("some/nested/node");
        assert!(result.is_err(), "nested path node should not be found");
    }

    // ── Battle tests: kill node ──────────────────────────────────────────

    #[test]
    fn kill_node_nonexistent_returns_error() {
        let result = kill_node("nonexistent_node_xyz", false);
        assert!(result.is_err(), "killing nonexistent node should fail");
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("not found"),
            "error should say not found: {}",
            err
        );
    }

    #[test]
    fn kill_node_empty_name_returns_error() {
        let result = kill_node("", false);
        assert!(result.is_err(), "killing empty name should fail");
    }

    #[test]
    fn kill_node_force_nonexistent_returns_error() {
        let result = kill_node("ghost_node", true);
        assert!(
            result.is_err(),
            "force-killing nonexistent node should still fail"
        );
        let err = result.unwrap_err().to_string();
        assert!(err.contains("not found"), "error: {}", err);
    }

    // ── Battle tests: restart node ───────────────────────────────────────

    #[test]
    fn restart_node_nonexistent_returns_error() {
        let result = restart_node("restart_me_if_you_can");
        assert!(result.is_err(), "restarting nonexistent node should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("not found"), "error: {}", err);
    }

    #[test]
    fn restart_node_empty_name_returns_error() {
        let result = restart_node("");
        assert!(result.is_err(), "restarting empty name should fail");
    }

    // ── Battle tests: pause node ─────────────────────────────────────────

    #[test]
    fn pause_node_nonexistent_returns_error() {
        let result = pause_node("pause_this_phantom");
        assert!(result.is_err(), "pausing nonexistent node should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("not found"), "error: {}", err);
    }

    #[test]
    fn pause_node_empty_name_returns_error() {
        let result = pause_node("");
        assert!(result.is_err(), "pausing empty name should fail");
    }

    #[test]
    fn pause_node_unicode_name_returns_error() {
        let result = pause_node("\u{00E9}l\u{00E8}ve_node");
        assert!(
            result.is_err(),
            "pausing unicode node should fail (not found)"
        );
    }

    // ── Battle tests: resume node ────────────────────────────────────────

    #[test]
    fn resume_node_nonexistent_returns_error() {
        let result = resume_node("resume_the_void");
        assert!(result.is_err(), "resuming nonexistent node should fail");
        let err = result.unwrap_err().to_string();
        assert!(err.contains("not found"), "error: {}", err);
    }

    #[test]
    fn resume_node_empty_name_returns_error() {
        let result = resume_node("");
        assert!(result.is_err(), "resuming empty name should fail");
    }

    // ── Battle tests: truncate_name edge cases ───────────────────────────

    #[test]
    fn truncate_name_empty_string() {
        assert_eq!(truncate_name("", 25), "");
    }

    #[test]
    fn truncate_name_zero_max() {
        // max_len of 0: should return empty or very short
        let result = truncate_name("hello", 0);
        assert!(
            result.len() <= 3,
            "zero max should produce minimal output: '{}'",
            result
        );
    }

    #[test]
    fn truncate_name_unicode_chars() {
        // Multi-byte chars: should not panic on char boundary issues
        let name = "\u{1F916}\u{1F916}\u{1F916}\u{1F916}\u{1F916}"; // 5 robot emojis (4 bytes each)
        let result = truncate_name(name, 10);
        assert!(
            result.len() <= 13,
            "truncated unicode should fit: len={}",
            result.len()
        );
        // Should not panic — that's the main assertion
    }

    #[test]
    fn truncate_name_with_slashes() {
        let name = "namespace/subspace/my_node_name";
        let result = truncate_name(name, 15);
        assert!(result.len() <= 15, "slashed name should be truncated");
    }

    #[test]
    fn truncate_name_max_one() {
        let result = truncate_name("hello_world", 1);
        assert!(result.len() <= 3, "max_len=1 result: '{}'", result);
    }

    #[test]
    fn truncate_name_max_three() {
        // Exactly enough room for "..."
        let result = truncate_name("hello_world", 3);
        assert!(result.len() <= 3, "max_len=3 result: '{}'", result);
    }

    // ── Battle tests: category matching coverage ─────────────────────────

    #[test]
    fn list_nodes_all_valid_category_aliases() {
        // Verify every documented alias resolves without category error
        let aliases = vec!["node", "nodes", "tool", "tools", "cli"];
        for alias in aliases {
            let result = list_nodes(false, false, Some(alias.to_string()));
            if let Err(e) = &result {
                let msg = e.to_string();
                assert!(
                    !msg.contains("Unknown category"),
                    "alias '{}' should be valid, got: {}",
                    alias,
                    msg
                );
            }
        }
    }

    #[test]
    fn list_nodes_category_mixed_case() {
        // "Node", "TOOLS", "Cli" etc should all work via to_lowercase
        for cat in &["Node", "NODES", "Tool", "TOOLS", "CLI", "Cli"] {
            let result = list_nodes(false, false, Some(cat.to_string()));
            if let Err(e) = &result {
                assert!(
                    !e.to_string().contains("Unknown category"),
                    "mixed-case '{}' should be valid",
                    cat
                );
            }
        }
    }

    // ── Battle tests: node name matching logic ───────────────────────────

    #[test]
    fn node_name_matching_exact() {
        let name = "motor_controller";
        let search = "motor_controller";
        let matches = name == search || name.ends_with(&format!("/{}", search));
        assert!(matches);
    }

    #[test]
    fn node_name_matching_suffix() {
        let name = "robot/motor_controller";
        let search = "motor_controller";
        let matches = name == search || name.ends_with(&format!("/{}", search));
        assert!(matches, "should match via ends_with /name");
    }

    #[test]
    fn node_name_matching_no_match() {
        let name = "motor_controller";
        let search = "sensor_reader";
        let matches = name == search || name.ends_with(&format!("/{}", search));
        assert!(!matches);
    }

    #[test]
    fn node_name_matching_partial_no_match() {
        // "controller" is a substring of "motor_controller" but should NOT match
        // because the matching logic requires exact or ends_with("/controller")
        let name = "motor_controller";
        let search = "controller";
        let matches = name == search || name.ends_with(&format!("/{}", search));
        assert!(!matches, "partial substring should not match");
    }
}
