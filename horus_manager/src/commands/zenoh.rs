//! Zenoh discovery and interaction commands
//!
//! Provides CLI tools for discovering and interacting with Zenoh topics.
//!
//! # Usage
//!
//! ```bash
//! # List all Zenoh topics
//! horus zenoh list
//!
//! # Show detailed topic info
//! horus zenoh info /scan
//!
//! # Echo messages from a topic
//! horus zenoh echo /scan
//!
//! # Publish a test message
//! horus zenoh pub /cmd_vel '{"linear": 0.5, "angular": 0.0}'
//! ```

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::collections::HashMap;
use std::time::{Duration, Instant};

/// Discovered Zenoh topic information
#[derive(Debug, Clone)]
pub struct ZenohTopicInfo {
    /// Topic key expression
    pub key_expr: String,
    /// Parsed topic name (without namespace prefix)
    pub topic_name: String,
    /// Whether it's a ROS2 topic (rt/ prefix)
    pub is_ros2: bool,
    /// Estimated message rate in Hz
    pub rate_hz: f64,
    /// Last message timestamp
    pub last_seen: Option<Instant>,
    /// Sample message size in bytes
    pub message_size: Option<usize>,
    /// Inferred message type (if detectable)
    pub message_type: Option<String>,
    /// Number of publishers
    pub publisher_count: usize,
    /// Number of subscribers
    pub subscriber_count: usize,
}


/// List all discovered Zenoh topics
///
/// Connects to the Zenoh network and discovers all available topics.
pub fn list_topics(verbose: bool, json: bool, ros2_only: bool) -> HorusResult<()> {
    // Create a tokio runtime for async Zenoh operations
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Config(format!("Failed to create async runtime: {}", e)))?;

    rt.block_on(async {
        list_topics_async(verbose, json, ros2_only).await
    })
}

async fn list_topics_async(verbose: bool, json: bool, ros2_only: bool) -> HorusResult<()> {
    println!("{}", "Connecting to Zenoh network...".dimmed());

    // Open a Zenoh session with default config
    let session = zenoh::open(zenoh::Config::default())
        .await
        .map_err(|e| HorusError::Config(format!("Failed to connect to Zenoh: {}", e)))?;

    println!("{}", "Discovering topics (5 seconds)...".dimmed());

    // Subscribe to all keys to discover topics
    let subscriber = session
        .declare_subscriber("**")
        .await
        .map_err(|e| HorusError::Config(format!("Failed to subscribe: {}", e)))?;

    // Collect topics for a few seconds
    let mut topics: HashMap<String, ZenohTopicInfo> = HashMap::new();
    let discovery_timeout = Duration::from_secs(5);
    let start = Instant::now();

    loop {
        tokio::select! {
            sample = subscriber.recv_async() => {
                if let Ok(sample) = sample {
                    let key_expr = sample.key_expr().as_str().to_string();

                    // Skip if filtering for ROS2 only
                    let is_ros2 = key_expr.starts_with("rt/") ||
                                  key_expr.starts_with("rq/") ||
                                  key_expr.starts_with("rr/");

                    if ros2_only && !is_ros2 {
                        continue;
                    }

                    let topic_name = if is_ros2 {
                        // Remove ROS2 prefix
                        key_expr.trim_start_matches("rt/")
                               .trim_start_matches("rq/")
                               .trim_start_matches("rr/")
                               .to_string()
                    } else {
                        key_expr.clone()
                    };

                    let payload_len = sample.payload().len();

                    let entry = topics.entry(key_expr.clone()).or_insert_with(|| {
                        ZenohTopicInfo {
                            key_expr: key_expr.clone(),
                            topic_name,
                            is_ros2,
                            rate_hz: 0.0,
                            last_seen: None,
                            message_size: Some(payload_len),
                            message_type: None,
                            publisher_count: 0,
                            subscriber_count: 0,
                        }
                    });

                    // Update rate estimation
                    if let Some(last) = entry.last_seen {
                        let elapsed = last.elapsed().as_secs_f64();
                        if elapsed > 0.0 {
                            // Exponential moving average
                            let instant_rate = 1.0 / elapsed;
                            entry.rate_hz = entry.rate_hz * 0.7 + instant_rate * 0.3;
                        }
                    }
                    entry.last_seen = Some(Instant::now());
                    entry.message_size = Some(payload_len);
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(100)) => {
                if start.elapsed() >= discovery_timeout {
                    break;
                }
            }
        }
    }

    // Close subscriber
    drop(subscriber);

    // Output results
    if json {
        let json_output = serde_json::to_string_pretty(
            &topics
                .values()
                .map(|t| {
                    serde_json::json!({
                        "key_expr": t.key_expr,
                        "topic_name": t.topic_name,
                        "is_ros2": t.is_ros2,
                        "rate_hz": t.rate_hz,
                        "message_size": t.message_size,
                        "message_type": t.message_type,
                    })
                })
                .collect::<Vec<_>>(),
        )
        .unwrap_or_default();
        println!("{}", json_output);
        return Ok(());
    }

    if topics.is_empty() {
        println!("{}", "No Zenoh topics discovered.".yellow());
        println!(
            "  {} Make sure there are active Zenoh publishers on the network",
            "Tip:".dimmed()
        );
        return Ok(());
    }

    println!();
    println!(
        "{}",
        format!("Discovered {} Zenoh topic(s):", topics.len())
            .green()
            .bold()
    );
    println!();

    if verbose {
        for topic in topics.values() {
            let ros2_badge = if topic.is_ros2 {
                "[ROS2]".magenta()
            } else {
                "[HORUS]".cyan()
            };
            println!(
                "  {} {} {}",
                ros2_badge,
                "Key:".dimmed(),
                topic.key_expr.white().bold()
            );
            println!("    {} {}", "Topic:".dimmed(), topic.topic_name);
            println!("    {} {:.1} Hz", "Rate:".dimmed(), topic.rate_hz);
            if let Some(size) = topic.message_size {
                println!("    {} {} bytes", "Size:".dimmed(), size);
            }
            if let Some(ref msg_type) = topic.message_type {
                println!("    {} {}", "Type:".dimmed(), msg_type);
            }
            println!();
        }
    } else {
        // Compact table view
        println!(
            "  {:<40} {:>10} {:>10} {:>8}",
            "KEY EXPRESSION".dimmed(),
            "TYPE".dimmed(),
            "SIZE".dimmed(),
            "RATE".dimmed()
        );
        println!("  {}", "-".repeat(72).dimmed());

        let mut sorted: Vec<_> = topics.values().collect();
        sorted.sort_by(|a, b| a.key_expr.cmp(&b.key_expr));

        for topic in sorted {
            let type_badge = if topic.is_ros2 { "ROS2" } else { "HORUS" };
            let size = topic
                .message_size
                .map(|s| format!("{} B", s))
                .unwrap_or_else(|| "-".to_string());
            let rate = format!("{:.1} Hz", topic.rate_hz);
            println!(
                "  {:<40} {:>10} {:>10} {:>8}",
                topic.key_expr, type_badge, size, rate
            );
        }
    }

    println!();
    session.close().await.ok();
    Ok(())
}

/// Show detailed information about a specific Zenoh topic
pub fn topic_info(key_expr: &str, json: bool) -> HorusResult<()> {
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Config(format!("Failed to create async runtime: {}", e)))?;

    rt.block_on(async { topic_info_async(key_expr, json).await })
}

async fn topic_info_async(key_expr: &str, json: bool) -> HorusResult<()> {
    println!("{}", "Connecting to Zenoh network...".dimmed());

    let session = zenoh::open(zenoh::Config::default())
        .await
        .map_err(|e| HorusError::Config(format!("Failed to connect to Zenoh: {}", e)))?;

    // Normalize key expression - handle ROS2-style topic names
    let normalized_key = if key_expr.starts_with('/') {
        format!("rt{}", key_expr)
    } else if !key_expr.contains('/') {
        format!("**/{}", key_expr)
    } else {
        key_expr.to_string()
    };

    println!(
        "{} {}",
        "Monitoring:".dimmed(),
        normalized_key.white().bold()
    );

    let subscriber = session
        .declare_subscriber(&normalized_key)
        .await
        .map_err(|e| HorusError::Config(format!("Failed to subscribe: {}", e)))?;

    // Collect statistics for a few seconds
    let mut message_count = 0u64;
    let mut total_bytes = 0u64;
    let mut min_size = usize::MAX;
    let mut max_size = 0usize;
    let mut timestamps: Vec<Instant> = Vec::new();
    let sample_duration = Duration::from_secs(3);
    let start = Instant::now();

    println!("{}", "Collecting statistics (3 seconds)...".dimmed());

    loop {
        tokio::select! {
            sample = subscriber.recv_async() => {
                if let Ok(sample) = sample {
                    let size = sample.payload().len();
                    message_count += 1;
                    total_bytes += size as u64;
                    min_size = min_size.min(size);
                    max_size = max_size.max(size);
                    timestamps.push(Instant::now());
                }
            }
            _ = tokio::time::sleep(Duration::from_millis(100)) => {
                if start.elapsed() >= sample_duration {
                    break;
                }
            }
        }
    }

    drop(subscriber);

    // Calculate rate from timestamps
    let rate_hz = if timestamps.len() >= 2 {
        let first = timestamps.first().unwrap();
        let last = timestamps.last().unwrap();
        let duration = last.duration_since(*first).as_secs_f64();
        if duration > 0.0 {
            (timestamps.len() - 1) as f64 / duration
        } else {
            0.0
        }
    } else {
        0.0
    };

    if json {
        let output = serde_json::json!({
            "key_expr": key_expr,
            "message_count": message_count,
            "total_bytes": total_bytes,
            "rate_hz": rate_hz,
            "min_size": if min_size == usize::MAX { 0 } else { min_size },
            "max_size": max_size,
            "avg_size": if message_count > 0 { total_bytes / message_count } else { 0 },
        });
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
    } else {
        println!();
        println!("{}", "Topic Information:".green().bold());
        println!();
        println!("  {} {}", "Key Expression:".cyan(), key_expr);
        println!(
            "  {} {} messages",
            "Messages received:".cyan(),
            message_count
        );
        println!(
            "  {} {} bytes",
            "Total bytes:".cyan(),
            total_bytes
        );
        println!("  {} {:.2} Hz", "Rate:".cyan(), rate_hz);

        if message_count > 0 {
            println!();
            println!("  {}", "Message Size Statistics:".yellow());
            println!(
                "    {} {} bytes",
                "Min:".dimmed(),
                if min_size == usize::MAX { 0 } else { min_size }
            );
            println!("    {} {} bytes", "Max:".dimmed(), max_size);
            println!(
                "    {} {} bytes",
                "Avg:".dimmed(),
                total_bytes / message_count
            );
        }

        if message_count == 0 {
            println!();
            println!(
                "  {}",
                "No messages received. Topic may be inactive.".yellow()
            );
        }
    }

    session.close().await.ok();
    Ok(())
}

/// Echo messages from a Zenoh topic
pub fn echo_topic(key_expr: &str, count: Option<usize>, raw: bool) -> HorusResult<()> {
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Config(format!("Failed to create async runtime: {}", e)))?;

    rt.block_on(async { echo_topic_async(key_expr, count, raw).await })
}

async fn echo_topic_async(key_expr: &str, count: Option<usize>, raw: bool) -> HorusResult<()> {
    println!("{}", "Connecting to Zenoh network...".dimmed());

    let session = zenoh::open(zenoh::Config::default())
        .await
        .map_err(|e| HorusError::Config(format!("Failed to connect to Zenoh: {}", e)))?;

    // Normalize key expression
    let normalized_key = if key_expr.starts_with('/') {
        format!("rt{}", key_expr)
    } else if !key_expr.contains('/') {
        format!("**/{}", key_expr)
    } else {
        key_expr.to_string()
    };

    println!(
        "{} {}",
        "Echoing:".green().bold(),
        normalized_key.white().bold()
    );
    println!("{}", "Press Ctrl+C to stop".dimmed());
    println!();

    let subscriber = session
        .declare_subscriber(&normalized_key)
        .await
        .map_err(|e| HorusError::Config(format!("Failed to subscribe: {}", e)))?;

    let mut received = 0usize;
    let max_count = count.unwrap_or(usize::MAX);

    loop {
        match subscriber.recv_async().await {
            Ok(sample) => {
                received += 1;
                let key = sample.key_expr().as_str();
                let payload = sample.payload().to_bytes();

                if raw {
                    // Print raw hex bytes
                    println!(
                        "[{}] {} ({} bytes):",
                        received,
                        key.cyan(),
                        payload.len()
                    );
                    print_hex_dump(&payload, 64);
                } else {
                    // Try to interpret as UTF-8 or JSON
                    let display = if let Ok(s) = std::str::from_utf8(&payload) {
                        // Try to pretty-print JSON
                        if let Ok(json) = serde_json::from_str::<serde_json::Value>(s) {
                            serde_json::to_string_pretty(&json).unwrap_or_else(|_| s.to_string())
                        } else {
                            s.to_string()
                        }
                    } else {
                        // Binary data - show summary
                        format!("<binary {} bytes>", payload.len())
                    };

                    println!("[{}] {}", received, key.cyan());
                    println!("  {}", display);
                    println!();
                }

                if received >= max_count {
                    break;
                }
            }
            Err(_) => {
                println!("{}", "Subscriber closed".yellow());
                break;
            }
        }
    }

    drop(subscriber);
    session.close().await.ok();
    Ok(())
}

/// Publish a message to a Zenoh topic
pub fn publish_topic(
    key_expr: &str,
    message: &str,
    rate: Option<f64>,
    count: Option<usize>,
) -> HorusResult<()> {
    let rt = tokio::runtime::Runtime::new()
        .map_err(|e| HorusError::Config(format!("Failed to create async runtime: {}", e)))?;

    rt.block_on(async { publish_topic_async(key_expr, message, rate, count).await })
}

async fn publish_topic_async(
    key_expr: &str,
    message: &str,
    rate: Option<f64>,
    count: Option<usize>,
) -> HorusResult<()> {
    println!("{}", "Connecting to Zenoh network...".dimmed());

    let session = zenoh::open(zenoh::Config::default())
        .await
        .map_err(|e| HorusError::Config(format!("Failed to connect to Zenoh: {}", e)))?;

    // Normalize key expression
    let normalized_key = if key_expr.starts_with('/') {
        format!("rt{}", key_expr)
    } else {
        key_expr.to_string()
    };

    // Parse message - try JSON first, then use as raw string
    let payload: Vec<u8> = if message.starts_with('{') || message.starts_with('[') {
        // JSON - validate and serialize
        let json: serde_json::Value = serde_json::from_str(message).map_err(|e| {
            HorusError::Config(format!("Invalid JSON message: {}", e))
        })?;
        serde_json::to_vec(&json).unwrap()
    } else {
        // Raw string
        message.as_bytes().to_vec()
    };

    let publisher = session
        .declare_publisher(&normalized_key)
        .await
        .map_err(|e| HorusError::Config(format!("Failed to declare publisher: {}", e)))?;

    let rate_hz = rate.unwrap_or(1.0);
    let max_count = count.unwrap_or(1);
    let interval = Duration::from_secs_f64(1.0 / rate_hz);

    println!(
        "{} {}",
        "Publishing to:".green().bold(),
        normalized_key.white().bold()
    );
    println!(
        "  {} {:.1} Hz, {} message(s)",
        "Rate:".dimmed(),
        rate_hz,
        max_count
    );
    println!();

    for i in 0..max_count {
        publisher
            .put(&payload[..])
            .await
            .map_err(|e| HorusError::Config(format!("Failed to publish: {}", e)))?;

        println!("[{}] Published {} bytes", i + 1, payload.len());

        if i + 1 < max_count {
            tokio::time::sleep(interval).await;
        }
    }

    println!();
    println!("{}", "Done.".green());

    drop(publisher);
    session.close().await.ok();
    Ok(())
}

/// Print a hex dump of binary data
fn print_hex_dump(data: &[u8], max_bytes: usize) {
    let display_len = data.len().min(max_bytes);
    for (i, chunk) in data[..display_len].chunks(16).enumerate() {
        let offset = i * 16;
        let hex: String = chunk
            .iter()
            .map(|b| format!("{:02x}", b))
            .collect::<Vec<_>>()
            .join(" ");
        let ascii: String = chunk
            .iter()
            .map(|&b| {
                if b.is_ascii_graphic() || b == b' ' {
                    b as char
                } else {
                    '.'
                }
            })
            .collect();
        println!("  {:04x}  {:<48}  {}", offset, hex, ascii);
    }
    if data.len() > max_bytes {
        println!("  ... ({} more bytes)", data.len() - max_bytes);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_hex_dump() {
        let data = b"Hello, World!";
        // Just verify it doesn't panic
        print_hex_dump(data, 64);
    }
}
