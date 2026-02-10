//! Topic command - Interact with HORUS topics
//!
//! Provides commands for listing, echoing, and publishing to topics.

use crate::discovery::discover_shared_memory;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::memory::shm_topics_dir;
use std::io::{Read, Write};
use std::time::{Duration, Instant};

/// List all active topics
pub fn list_topics(verbose: bool, json: bool) -> HorusResult<()> {
    let topics = discover_shared_memory()?;

    if json {
        let json_output = serde_json::to_string_pretty(
            &topics
                .iter()
                .map(|t| {
                    serde_json::json!({
                        "name": t.topic_name,
                        "size_bytes": t.size_bytes,
                        "active": t.active,
                        "message_type": t.message_type,
                        "publishers": t.publishers,
                        "subscribers": t.subscribers,
                        "rate_hz": t.message_rate_hz
                    })
                })
                .collect::<Vec<_>>(),
        )
        .unwrap_or_default();
        println!("{}", json_output);
        return Ok(());
    }

    if topics.is_empty() {
        println!("{}", "No active topics found.".yellow());
        println!(
            "  {} Start a HORUS application to create topics",
            "Tip:".dimmed()
        );
        return Ok(());
    }

    println!("{}", "Active Topics:".green().bold());
    println!();

    if verbose {
        for topic in &topics {
            println!("  {} {}", "Topic:".cyan(), topic.topic_name.white().bold());
            println!("    {} {} bytes", "Size:".dimmed(), topic.size_bytes);
            println!(
                "    {} {}",
                "Active:".dimmed(),
                if topic.active {
                    "Yes".green()
                } else {
                    "No".red()
                }
            );
            if let Some(ref msg_type) = topic.message_type {
                println!("    {} {}", "Type:".dimmed(), msg_type);
            }
            println!("    {} {:.1} Hz", "Rate:".dimmed(), topic.message_rate_hz);
            if !topic.publishers.is_empty() {
                println!(
                    "    {} {}",
                    "Publishers:".dimmed(),
                    topic.publishers.join(", ")
                );
            }
            if !topic.subscribers.is_empty() {
                println!(
                    "    {} {}",
                    "Subscribers:".dimmed(),
                    topic.subscribers.join(", ")
                );
            }
            println!();
        }
    } else {
        // Compact table view
        println!(
            "  {:<30} {:>10} {:>8} {:>12}",
            "NAME".dimmed(),
            "SIZE".dimmed(),
            "RATE".dimmed(),
            "STATUS".dimmed()
        );
        println!("  {}", "-".repeat(64).dimmed());

        for topic in &topics {
            let status = if topic.active {
                "active".green()
            } else {
                "inactive".red()
            };
            let size = format_bytes(topic.size_bytes);
            let rate = format!("{:.1} Hz", topic.message_rate_hz);
            println!(
                "  {:<30} {:>10} {:>8} {:>12}",
                topic.topic_name, size, rate, status
            );
        }
    }

    println!();
    println!("  {} {} topic(s)", "Total:".dimmed(), topics.len());

    Ok(())
}

/// Echo messages from a topic
pub fn echo_topic(name: &str, count: Option<usize>, rate: Option<f64>) -> HorusResult<()> {
    let topics = discover_shared_memory()?;

    // Find the topic - match by exact name, path suffix, or base name
    let topic = topics.iter().find(|t| {
        t.topic_name == name
            || t.topic_name.ends_with(&format!("/{}", name))
            || t.topic_name
                .rsplit('/')
                .next()
                .map(|base| base == name)
                .unwrap_or(false)
    });

    if topic.is_none() {
        return Err(HorusError::Config(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        )));
    }

    let topic = topic.unwrap();
    let topic_path = shm_topics_dir().join(&topic.topic_name);

    println!(
        "{} Echoing topic: {}",
        "".cyan(),
        topic.topic_name.white().bold()
    );
    if let Some(ref msg_type) = topic.message_type {
        println!("  {} {}", "Type:".dimmed(), msg_type);
    }
    println!("  {} Press Ctrl+C to stop", "".dimmed());
    println!();

    let sleep_duration = rate
        .map(|r| Duration::from_secs_f64(1.0 / r))
        .unwrap_or(Duration::from_millis(100));
    let mut messages_received = 0;
    let mut last_content: Option<Vec<u8>> = None;

    loop {
        // Check if we've received enough messages
        if let Some(max_count) = count {
            if messages_received >= max_count {
                break;
            }
        }

        // Read the topic file
        if topic_path.exists() {
            if let Ok(mut file) = std::fs::File::open(&topic_path) {
                let mut content = Vec::new();
                if file.read_to_end(&mut content).is_ok() && !content.is_empty() {
                    // Only print if content changed
                    if last_content.as_ref() != Some(&content) {
                        messages_received += 1;
                        print_message(&content, messages_received);
                        last_content = Some(content);
                    }
                }
            }
        }

        std::thread::sleep(sleep_duration);
    }

    println!();
    println!("{} Received {} message(s)", "".green(), messages_received);

    Ok(())
}

/// Print a message in a readable format
fn print_message(data: &[u8], seq: usize) {
    let timestamp = chrono::Local::now().format("%H:%M:%S%.3f");

    // Try to interpret as text first
    if let Ok(text) = std::str::from_utf8(data) {
        if text
            .chars()
            .all(|c| c.is_ascii_graphic() || c.is_ascii_whitespace())
        {
            println!("[{}] #{}: {}", timestamp.to_string().dimmed(), seq, text);
            return;
        }
    }

    // Try to parse as JSON
    if let Ok(json) = serde_json::from_slice::<serde_json::Value>(data) {
        println!("[{}] #{}:", timestamp.to_string().dimmed(), seq);
        println!(
            "{}",
            serde_json::to_string_pretty(&json).unwrap_or_default()
        );
        return;
    }

    // Fall back to hex dump for binary data
    println!(
        "[{}] #{}: {} bytes",
        timestamp.to_string().dimmed(),
        seq,
        data.len()
    );
    print_hex_dump(data, 32);
}

/// Print hex dump of binary data
fn print_hex_dump(data: &[u8], max_bytes: usize) {
    let bytes_to_show = data.len().min(max_bytes);
    let hex: String = data[..bytes_to_show]
        .iter()
        .map(|b| format!("{:02x}", b))
        .collect::<Vec<_>>()
        .join(" ");

    print!("  {}", hex.dimmed());
    if data.len() > max_bytes {
        print!(
            " {} ... ({} more bytes)",
            "".dimmed(),
            data.len() - max_bytes
        );
    }
    println!();
}

/// Get detailed info about a topic
pub fn topic_info(name: &str) -> HorusResult<()> {
    let topics = discover_shared_memory()?;

    // Match by exact name, path suffix, or base name
    let topic = topics.iter().find(|t| {
        t.topic_name == name
            || t.topic_name.ends_with(&format!("/{}", name))
            || t.topic_name
                .rsplit('/')
                .next()
                .map(|base| base == name)
                .unwrap_or(false)
    });

    if topic.is_none() {
        return Err(HorusError::Config(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        )));
    }

    let topic = topic.unwrap();

    println!("{}", "Topic Information".green().bold());
    println!();
    println!("  {} {}", "Name:".cyan(), topic.topic_name.white().bold());
    println!("  {} {} bytes", "Size:".cyan(), topic.size_bytes);
    println!(
        "  {} {}",
        "Active:".cyan(),
        if topic.active {
            "Yes".green()
        } else {
            "No".red()
        }
    );

    if let Some(ref msg_type) = topic.message_type {
        println!("  {} {}", "Message Type:".cyan(), msg_type);
    }

    println!("  {} {:.2} Hz", "Rate:".cyan(), topic.message_rate_hz);

    if let Some(modified) = topic.last_modified {
        if let Ok(duration) = modified.elapsed() {
            println!(
                "  {} {:.1}s ago",
                "Last Update:".cyan(),
                duration.as_secs_f64()
            );
        }
    }

    println!();
    println!("  {}", "Publishers:".cyan());
    if topic.publishers.is_empty() {
        println!("    {}", "(none)".dimmed());
    } else {
        for pub_name in &topic.publishers {
            println!("    - {}", pub_name);
        }
    }

    println!();
    println!("  {}", "Subscribers:".cyan());
    if topic.subscribers.is_empty() {
        println!("    {}", "(none)".dimmed());
    } else {
        for sub_name in &topic.subscribers {
            println!("    - {}", sub_name);
        }
    }

    Ok(())
}

/// Measure topic publish rate
pub fn topic_hz(name: &str, window: Option<usize>) -> HorusResult<()> {
    let topics = discover_shared_memory()?;

    // Match by exact name, path suffix, or base name
    let topic = topics.iter().find(|t| {
        t.topic_name == name
            || t.topic_name.ends_with(&format!("/{}", name))
            || t.topic_name
                .rsplit('/')
                .next()
                .map(|base| base == name)
                .unwrap_or(false)
    });

    if topic.is_none() {
        return Err(HorusError::Config(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        )));
    }

    let topic = topic.unwrap();
    let topic_path = shm_topics_dir().join(&topic.topic_name);
    let window_size = window.unwrap_or(10);

    println!(
        "{} Measuring rate for: {}",
        "".cyan(),
        topic.topic_name.white().bold()
    );
    println!("  {} Press Ctrl+C to stop", "".dimmed());
    println!();

    let mut timestamps: Vec<Instant> = Vec::with_capacity(window_size);
    let mut last_content: Option<Vec<u8>> = None;

    loop {
        if topic_path.exists() {
            if let Ok(mut file) = std::fs::File::open(&topic_path) {
                let mut content = Vec::new();
                if file.read_to_end(&mut content).is_ok()
                    && !content.is_empty()
                    && last_content.as_ref() != Some(&content)
                {
                    timestamps.push(Instant::now());
                    last_content = Some(content);

                    // Keep only window_size timestamps
                    if timestamps.len() > window_size {
                        timestamps.remove(0);
                    }

                    // Calculate rate
                    if timestamps.len() >= 2 {
                        let duration = timestamps
                            .last()
                            .unwrap()
                            .duration_since(*timestamps.first().unwrap());
                        let rate = (timestamps.len() - 1) as f64 / duration.as_secs_f64();

                        print!(
                            "\r  {} {:.2} Hz (window: {})    ",
                            "Rate:".cyan(),
                            rate,
                            timestamps.len()
                        );
                        std::io::stdout().flush().ok();
                    }
                }
            }
        }

        std::thread::sleep(Duration::from_millis(10));
    }
}

/// Publish a message to a topic (for testing)
pub fn publish_topic(
    name: &str,
    message: &str,
    rate: Option<f64>,
    count: Option<usize>,
) -> HorusResult<()> {
    let topic_path = shm_topics_dir().join(name);

    // Create topic directory if needed
    if let Some(parent) = topic_path.parent() {
        std::fs::create_dir_all(parent).ok();
    }

    let sleep_duration = rate.map(|r| Duration::from_secs_f64(1.0 / r));
    let publish_count = count.unwrap_or(1);

    println!("{} Publishing to: {}", "".cyan(), name.white().bold());

    for i in 0..publish_count {
        // Write message to topic file
        if let Ok(mut file) = std::fs::File::create(&topic_path) {
            file.write_all(message.as_bytes())?;
            println!(
                "  [{}] Published: {}",
                i + 1,
                message.chars().take(50).collect::<String>()
            );
        }

        if let Some(duration) = sleep_duration {
            if i < publish_count - 1 {
                std::thread::sleep(duration);
            }
        }
    }

    println!();
    println!("{} Published {} message(s)", "".green(), publish_count);

    Ok(())
}

/// Format bytes in human-readable form
fn format_bytes(bytes: u64) -> String {
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    }
}

#[derive(Debug, Clone)]
pub struct TopicInfo {
    pub name: String,
    pub message_type: Option<String>,
}
