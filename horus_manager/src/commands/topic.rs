//! Topic command - Interact with HORUS topics
//!
//! Provides commands for listing, echoing, and publishing to topics.

use crate::cli_output;
use crate::discovery::discover_shared_memory;
use crate::progress::format_bytes;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::memory::shm_topics_dir;
use std::io::Write;
use std::time::{Duration, Instant};

/// List all active topics
pub fn list_topics(verbose: bool, json: bool) -> HorusResult<()> {
    let topics = discover_shared_memory()?;

    if json {
        let items: Vec<_> = topics
            .iter()
            .map(|t| {
                serde_json::json!({
                    "name": t.topic_name,
                    "size_bytes": t.size_bytes,
                    "active": t.active,
                    "message_type": t.message_type,
                    "publishers": t.publishers,
                    "subscribers": t.subscribers,
                    "rate_hz": t.message_rate_hz,
                    "status": format!("{:?}", t.status),
                    "age": t.age_string
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
///
/// Reads the ring buffer directly from the topic's shared-memory backing file
/// instead of reading raw file bytes.  Only the payload region of the most
/// recently written slot is inspected, so the output is actual message data
/// rather than binary ring-buffer internals.
pub fn echo_topic(name: &str, count: Option<usize>, rate: Option<f64>) -> HorusResult<()> {
    use horus_core::communication::read_latest_slot_bytes;

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

    let Some(topic) = topic else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        ))));
    };
    let topic_path = shm_topics_dir().join(&topic.topic_name);

    println!(
        "{} Echoing topic: {}",
        cli_output::ICON_INFO.cyan(),
        topic.topic_name.white().bold()
    );
    if let Some(ref msg_type) = topic.message_type {
        println!("  {} {}", "Type:".dimmed(), msg_type);
    }
    println!("  {} Press Ctrl+C to stop", "".dimmed());
    println!();

    let sleep_duration = match rate {
        Some(r) if r <= 0.0 => {
            return Err(HorusError::Config(ConfigError::Other(
                "Rate must be greater than 0.0".to_string(),
            )));
        }
        Some(r) => Duration::from_secs_f64(1.0 / r),
        None => Duration::from_millis(100),
    };

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    let mut messages_received = 0usize;
    // Track the write_idx so we only print when a new message is actually published.
    let mut last_write_idx: u64 = 0;

    loop {
        if !running.load(std::sync::atomic::Ordering::SeqCst) {
            break;
        }
        if let Some(max_count) = count {
            if messages_received >= max_count {
                break;
            }
        }

        if let Some(slot) = read_latest_slot_bytes(&topic_path, last_write_idx) {
            last_write_idx = slot.write_idx;
            messages_received += 1;
            print_message(&slot.payload, messages_received, slot.is_pod);
        }

        std::thread::sleep(sleep_duration);
    }

    println!();
    println!(
        "{} Received {} message(s)",
        cli_output::ICON_SUCCESS.green(),
        messages_received
    );

    Ok(())
}

/// Print message payload in a readable format.
///
/// For POD types the bytes are raw struct data.
/// For serde types the bytes are bincode-encoded; we try JSON (if the type
/// happens to be JSON-compatible via a round-trip through `serde_json`),
/// then fall back to a bincode hex dump showing the actual serialized form.
fn print_message(data: &[u8], seq: usize, is_pod: bool) {
    let timestamp = chrono::Local::now().format("%H:%M:%S%.3f");

    if is_pod {
        // POD types: raw struct bytes — try text, then hex
        if let Ok(text) = std::str::from_utf8(data) {
            if text
                .chars()
                .all(|c| c.is_ascii_graphic() || c.is_ascii_whitespace())
            {
                println!("[{}] #{}: {}", timestamp.to_string().dimmed(), seq, text);
                return;
            }
        }
        println!(
            "[{}] #{}: {} bytes (POD)",
            timestamp.to_string().dimmed(),
            seq,
            data.len()
        );
        print_hex_dump(data, 64);
        return;
    }

    // Non-POD / serde (bincode): attempt a JSON display via serde_json.
    // Many robotics message types use field-by-field struct layout that maps
    // cleanly to JSON when re-encoded.
    if let Ok(json) = serde_json::from_slice::<serde_json::Value>(data) {
        // The payload happened to be valid JSON (e.g., JSON-serialized types).
        println!("[{}] #{}:", timestamp.to_string().dimmed(), seq);
        println!(
            "{}",
            serde_json::to_string_pretty(&json).unwrap_or_default()
        );
        return;
    }

    // bincode payload — show the raw bytes as a hex dump.
    println!(
        "[{}] #{}: {} bytes (bincode)",
        timestamp.to_string().dimmed(),
        seq,
        data.len()
    );
    print_hex_dump(data, 64);
}

/// Print hex dump of binary data (up to `max_bytes`).
fn print_hex_dump(data: &[u8], max_bytes: usize) {
    let bytes_to_show = data.len().min(max_bytes);
    let hex: String = data[..bytes_to_show]
        .chunks(16)
        .map(|row| {
            row.iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n  ");

    print!("  {}", hex.dimmed());
    if data.len() > max_bytes {
        print!(
            "\n  {} ... ({} more bytes)",
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

    let Some(topic) = topic else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        ))));
    };

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
    use horus_core::communication::read_latest_slot_bytes;

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

    let Some(topic) = topic else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        ))));
    };
    let topic_path = shm_topics_dir().join(&topic.topic_name);
    let window_size = window.unwrap_or(10);
    if window_size == 0 {
        return Err(HorusError::Config(ConfigError::Other(
            "--window must be at least 1".to_string(),
        )));
    }

    println!(
        "{} Measuring rate for: {}",
        cli_output::ICON_INFO.cyan(),
        topic.topic_name.white().bold()
    );
    println!("  {} Press Ctrl+C to stop", "".dimmed());
    println!();

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    let mut timestamps: std::collections::VecDeque<Instant> =
        std::collections::VecDeque::with_capacity(window_size + 1);
    let mut last_write_idx: u64 = 0;
    let mut total_messages: u64 = 0;
    let start_time = Instant::now();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        if let Some(slot) = read_latest_slot_bytes(&topic_path, last_write_idx) {
            last_write_idx = slot.write_idx;
            timestamps.push_back(Instant::now());
            total_messages += 1;

            // Keep only window_size timestamps
            while timestamps.len() > window_size {
                timestamps.pop_front();
            }

            // Calculate rate (need at least 2 timestamps)
            if timestamps.len() >= 2 {
                let first = timestamps.front().unwrap();
                let last = timestamps.back().unwrap();
                let duration_secs = last.duration_since(*first).as_secs_f64();

                if duration_secs > 0.0 {
                    let rate = (timestamps.len() - 1) as f64 / duration_secs;
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

        std::thread::sleep(Duration::from_millis(10));
    }

    // Print final stats
    println!();
    let elapsed = start_time.elapsed().as_secs_f64();
    if total_messages > 0 && elapsed > 0.0 {
        let avg_rate = total_messages as f64 / elapsed;
        println!(
            "{} Average rate: {:.2} Hz ({} messages in {:.1}s)",
            cli_output::ICON_INFO.cyan(),
            avg_rate,
            total_messages,
            elapsed
        );
    } else {
        println!("{} No messages received", cli_output::ICON_INFO.dimmed());
    }
    Ok(())
}

/// Measure topic bandwidth (bytes/sec and messages/sec)
///
/// Equivalent to `ros2 topic bw`.
pub fn topic_bw(name: &str, window: Option<usize>) -> HorusResult<()> {
    use horus_core::communication::read_latest_slot_bytes;

    let topics = discover_shared_memory()?;

    let topic = topics.iter().find(|t| {
        t.topic_name == name
            || t.topic_name.ends_with(&format!("/{}", name))
            || t.topic_name
                .rsplit('/')
                .next()
                .map(|base| base == name)
                .unwrap_or(false)
    });

    let Some(topic) = topic else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Topic '{}' not found. Use 'horus topic list' to see available topics.",
            name
        ))));
    };
    let topic_path = shm_topics_dir().join(&topic.topic_name);
    let window_size = window.unwrap_or(100);
    if window_size == 0 {
        return Err(HorusError::Config(ConfigError::Other(
            "--window must be at least 1".to_string(),
        )));
    }

    println!(
        "{} Measuring bandwidth for: {}",
        cli_output::ICON_INFO.cyan(),
        topic.topic_name.white().bold()
    );
    println!("  {} Press Ctrl+C to stop", "".dimmed());
    println!();

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    // Each sample: (timestamp, bytes_in_slot)
    let mut samples: std::collections::VecDeque<(Instant, usize)> =
        std::collections::VecDeque::with_capacity(window_size + 1);
    let mut last_write_idx: u64 = 0;

    loop {
        if !running.load(std::sync::atomic::Ordering::SeqCst) {
            break;
        }

        if let Some(slot) = read_latest_slot_bytes(&topic_path, last_write_idx) {
            last_write_idx = slot.write_idx;
            samples.push_back((Instant::now(), slot.payload.len()));

            while samples.len() > window_size {
                samples.pop_front();
            }

            if samples.len() >= 2 {
                let first = &samples[0];
                let last = &samples[samples.len() - 1];
                let duration_secs = last.0.duration_since(first.0).as_secs_f64();

                if duration_secs > 0.0 {
                    let total_bytes: usize = samples.iter().map(|(_, b)| b).sum();
                    let bytes_per_sec = total_bytes as f64 / duration_secs;
                    let msgs_per_sec = (samples.len() - 1) as f64 / duration_secs;

                    let (bw_val, bw_unit) = if bytes_per_sec >= 1_000_000.0 {
                        (bytes_per_sec / 1_000_000.0, "MB/s")
                    } else if bytes_per_sec >= 1_000.0 {
                        (bytes_per_sec / 1_000.0, "KB/s")
                    } else {
                        (bytes_per_sec, "B/s")
                    };

                    let avg_bytes = total_bytes / samples.len();

                    print!(
                        "\r  {} {:.2} {} ({:.1} msgs/s, avg {}/msg, window: {})    ",
                        "Bandwidth:".cyan(),
                        bw_val,
                        bw_unit,
                        msgs_per_sec,
                        format_bytes(avg_bytes as u64),
                        samples.len()
                    );
                    std::io::stdout().flush().ok();
                }
            }
        }

        std::thread::sleep(Duration::from_millis(10));
    }

    println!();
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn print_hex_dump_small_payload() {
        // Just verify it doesn't panic on various sizes
        print_hex_dump(&[], 64);
        print_hex_dump(&[0xAA], 64);
        print_hex_dump(&[0x01, 0x02, 0x03, 0x04], 64);
    }

    #[test]
    fn print_hex_dump_truncated() {
        let data: Vec<u8> = (0..128).collect();
        // Should not panic, truncates to max_bytes
        print_hex_dump(&data, 16);
    }

    #[test]
    fn print_message_pod_text() {
        // ASCII text should print as text, not crash
        print_message(b"hello world", 1, true);
    }

    #[test]
    fn print_message_pod_binary() {
        // Non-ASCII POD should fall back to hex
        print_message(&[0xFF, 0x00, 0x01], 1, true);
    }

    #[test]
    fn print_message_serde_valid_json() {
        // Valid JSON payload
        print_message(b"{\"x\":1.0}", 1, false);
    }

    #[test]
    fn print_message_serde_non_json() {
        // Non-JSON serde (bincode-like) should fall back to hex
        print_message(
            &[0x05, 0x00, 0x00, 0x00, b'h', b'e', b'l', b'l', b'o'],
            1,
            false,
        );
    }
}

/// Publish a message to a topic (for testing)
pub fn publish_topic(
    name: &str,
    message: &str,
    rate: Option<f64>,
    count: Option<usize>,
) -> HorusResult<()> {
    use horus_core::communication::Topic;

    // Parse JSON message
    let value: serde_json::Value = serde_json::from_str(message).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Invalid JSON message: {}. Example: '{{\"linear\": 1.0}}'",
            e
        )))
    })?;

    // Create topic using the proper ring buffer protocol
    let topic: Topic<serde_json::Value> = Topic::new(name).map_err(|e| {
        HorusError::Communication(horus_core::error::CommunicationError::TopicCreationFailed {
            topic: name.to_string(),
            reason: e.to_string(),
        })
    })?;

    let sleep_duration = match rate {
        Some(r) if r <= 0.0 => {
            return Err(HorusError::Config(ConfigError::Other(
                "Rate must be greater than 0.0".to_string(),
            )));
        }
        Some(r) => Some(Duration::from_secs_f64(1.0 / r)),
        None => None,
    };
    let publish_count = count.unwrap_or(1);

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    println!(
        "{} Publishing to: {}",
        cli_output::ICON_INFO.cyan(),
        name.white().bold()
    );

    let mut published = 0;
    for i in 0..publish_count {
        if !running.load(std::sync::atomic::Ordering::SeqCst) {
            break;
        }
        topic.send(value.clone());
        published += 1;
        println!(
            "  [{}] Published: {}",
            i + 1,
            message.chars().take(50).collect::<String>()
        );

        if let Some(duration) = sleep_duration {
            if i < publish_count - 1 {
                std::thread::sleep(duration);
            }
        }
    }

    println!();
    println!(
        "{} Published {} message(s)",
        cli_output::ICON_SUCCESS.green(),
        published
    );

    Ok(())
}
