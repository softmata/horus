//! Discovery command - Discover HORUS nodes on the network via mDNS
//!
//! Provides commands for discovering HORUS nodes using mDNS/DNS-SD.
//! Supports one-shot scanning and continuous watching modes.

use colored::*;
use horus_core::communication::network::{
    discover_full_with_options, watch_with_interval, DiscoveredNode, DiscoveryEvent,
    DiscoveryOptions,
};
use horus_core::error::HorusResult;
use std::time::Duration;

/// Run the discover command
pub fn run_discover(
    timeout: u64,
    topic: Option<String>,
    name: Option<String>,
    watch: bool,
    format: &str,
) -> HorusResult<()> {
    if watch {
        watch_nodes(topic, name, format)
    } else {
        scan_nodes(timeout, topic, name, format)
    }
}

/// Scan for nodes once and display results
fn scan_nodes(
    timeout: u64,
    topic: Option<String>,
    name: Option<String>,
    format: &str,
) -> HorusResult<()> {
    let options = DiscoveryOptions {
        timeout: Duration::from_secs(timeout),
        filter_topic: topic.clone(),
        filter_name: name.clone(),
        include_cached: false,
    };

    // Show scanning message for non-JSON output
    if format != "json" {
        println!(
            "{} Scanning for HORUS nodes ({} seconds)...",
            "🔍".cyan(),
            timeout
        );
        if let Some(ref t) = topic {
            println!("  {} topic: {}", "Filter:".dimmed(), t);
        }
        if let Some(ref n) = name {
            println!("  {} name: {}", "Filter:".dimmed(), n);
        }
        println!();
    }

    let result = discover_full_with_options(options)?;

    match format {
        "json" => {
            println!(
                "{}",
                serde_json::to_string_pretty(&result).unwrap_or_default()
            );
        }
        "simple" => {
            print_nodes_simple(&result.nodes);
        }
        _ => {
            // Default table format
            print_nodes_table(&result.nodes);
            println!();
            println!(
                "  {} {} node(s) found in {}ms",
                "Total:".dimmed(),
                result.count,
                result.scan_duration_ms
            );
        }
    }

    Ok(())
}

/// Continuously watch for nodes joining and leaving
fn watch_nodes(topic: Option<String>, name: Option<String>, format: &str) -> HorusResult<()> {
    if format != "json" {
        println!("{} Watching for HORUS nodes (Ctrl+C to stop)...", "👁".cyan());
        if let Some(ref t) = topic {
            println!("  {} topic: {}", "Filter:".dimmed(), t);
        }
        if let Some(ref n) = name {
            println!("  {} name: {}", "Filter:".dimmed(), n);
        }
        println!();
    }

    let watcher = watch_with_interval(Duration::from_secs(1))?;

    for event in watcher {
        // Apply filters
        match &event {
            DiscoveryEvent::NodeJoined(node) | DiscoveryEvent::NodeUpdated(node) => {
                if let Some(ref t) = topic {
                    if !node.topics.iter().any(|topic_name| topic_name.contains(t)) {
                        continue;
                    }
                }
                if let Some(ref n) = name {
                    if !node.name.contains(n) && !node.hostname.contains(n) {
                        continue;
                    }
                }
            }
            DiscoveryEvent::NodeLeft(node_name) => {
                if let Some(ref n) = name {
                    if !node_name.contains(n) {
                        continue;
                    }
                }
                // Can't filter by topic for left events since we don't have the node info
            }
            DiscoveryEvent::Error(_) => {}
        }

        match format {
            "json" => print_event_json(&event),
            _ => print_event_colored(&event),
        }
    }

    Ok(())
}

/// Print nodes in table format
fn print_nodes_table(nodes: &[DiscoveredNode]) {
    if nodes.is_empty() {
        println!("{}", "No HORUS nodes found on the network.".yellow());
        println!(
            "  {} Make sure nodes are running with mDNS enabled",
            "Tip:".dimmed()
        );
        println!(
            "  {} Check that your firewall allows mDNS (port 5353 UDP)",
            "    ".dimmed()
        );
        return;
    }

    println!("{}", "Discovered Nodes:".green().bold());
    println!();

    // Table header
    println!(
        "  {:<20} {:<16} {:>6} {:<30}",
        "NAME".dimmed(),
        "ADDRESS".dimmed(),
        "PORT".dimmed(),
        "TOPICS".dimmed()
    );
    println!("  {}", "-".repeat(76).dimmed());

    for node in nodes {
        let topics_str = if node.topics.is_empty() {
            "(none)".dimmed().to_string()
        } else if node.topics.len() <= 2 {
            node.topics.join(", ")
        } else {
            format!("{}, ... (+{})", node.topics[0], node.topics.len() - 1)
        };

        println!(
            "  {:<20} {:<16} {:>6} {:<30}",
            truncate_str(&node.name, 20).white().bold(),
            node.ip,
            node.port,
            topics_str
        );
    }
}

/// Print nodes in simple format (one per line)
fn print_nodes_simple(nodes: &[DiscoveredNode]) {
    for node in nodes {
        println!("{}:{}  {}", node.ip, node.port, node.name);
    }
}

/// Print event in JSON format
fn print_event_json(event: &DiscoveryEvent) {
    let json = match event {
        DiscoveryEvent::NodeJoined(node) => {
            serde_json::json!({
                "event": "joined",
                "node": node
            })
        }
        DiscoveryEvent::NodeLeft(name) => {
            serde_json::json!({
                "event": "left",
                "name": name
            })
        }
        DiscoveryEvent::NodeUpdated(node) => {
            serde_json::json!({
                "event": "updated",
                "node": node
            })
        }
        DiscoveryEvent::Error(msg) => {
            serde_json::json!({
                "event": "error",
                "message": msg
            })
        }
    };
    println!("{}", serde_json::to_string(&json).unwrap_or_default());
}

/// Print event with colors
fn print_event_colored(event: &DiscoveryEvent) {
    let timestamp = chrono::Local::now().format("%H:%M:%S");

    match event {
        DiscoveryEvent::NodeJoined(node) => {
            println!(
                "[{}] {} {} ({}:{})",
                timestamp.to_string().dimmed(),
                "+".green().bold(),
                node.name.white().bold(),
                node.ip,
                node.port
            );
            if !node.topics.is_empty() {
                println!(
                    "         {} {}",
                    "Topics:".dimmed(),
                    node.topics.join(", ")
                );
            }
        }
        DiscoveryEvent::NodeLeft(name) => {
            println!(
                "[{}] {} {}",
                timestamp.to_string().dimmed(),
                "-".red().bold(),
                name
            );
        }
        DiscoveryEvent::NodeUpdated(node) => {
            println!(
                "[{}] {} {} ({}:{})",
                timestamp.to_string().dimmed(),
                "~".yellow().bold(),
                node.name.white().bold(),
                node.ip,
                node.port
            );
        }
        DiscoveryEvent::Error(msg) => {
            println!(
                "[{}] {} {}",
                timestamp.to_string().dimmed(),
                "!".red().bold(),
                msg.red()
            );
        }
    }
}

/// Truncate a string to max length with ellipsis
fn truncate_str(s: &str, max_len: usize) -> String {
    if s.len() <= max_len {
        s.to_string()
    } else {
        format!("{}...", &s[..max_len.saturating_sub(3)])
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_truncate_str() {
        assert_eq!(truncate_str("hello", 10), "hello");
        assert_eq!(truncate_str("hello world", 8), "hello...");
        assert_eq!(truncate_str("hi", 2), "hi");
    }
}
