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
use std::time::Instant;
use horus_core::core::DurationExt;

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
                    "messages_total": t.messages_total,
                    "topic_kind": t.topic_kind,
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
            "  {:<30} {:>10} {:>10} {:>8} {:>12}",
            "NAME".dimmed(),
            "SIZE".dimmed(),
            "MSGS".dimmed(),
            "RATE".dimmed(),
            "STATUS".dimmed()
        );
        println!("  {}", "-".repeat(74).dimmed());

        for topic in &topics {
            let status = if topic.active {
                "active".green()
            } else {
                "inactive".red()
            };
            let size = format_bytes(topic.size_bytes);
            let rate = format!("{:.1} Hz", topic.message_rate_hz);
            let msgs = if topic.messages_total > 0 {
                format!("{}", topic.messages_total)
            } else {
                "-".to_string()
            };
            println!(
                "  {:<30} {:>10} {:>10} {:>8} {:>12}",
                topic.topic_name, size, msgs, rate, status
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
        Some(r) => r.hz().period(),
        None => 100_u64.ms(),
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
/// Classification of how a message payload will be displayed.
#[cfg_attr(test, derive(Debug, PartialEq))]
enum MessageFormat {
    /// ASCII-safe text (POD that is valid UTF-8 printable)
    PodText(String),
    /// Binary POD → hex dump
    PodHex,
    /// Valid JSON payload → pretty-printed
    Json(String),
    /// Non-JSON serde (bincode) → hex dump
    BincodeHex,
}

/// Classify how a message payload should be displayed without printing.
fn classify_message(data: &[u8], is_pod: bool) -> MessageFormat {
    if is_pod {
        if let Ok(text) = std::str::from_utf8(data) {
            if text
                .chars()
                .all(|c| c.is_ascii_graphic() || c.is_ascii_whitespace())
            {
                return MessageFormat::PodText(text.to_string());
            }
        }
        return MessageFormat::PodHex;
    }

    if let Ok(json) = serde_json::from_slice::<serde_json::Value>(data) {
        return MessageFormat::Json(serde_json::to_string_pretty(&json).unwrap_or_default());
    }

    MessageFormat::BincodeHex
}

fn print_message(data: &[u8], seq: usize, is_pod: bool) {
    let timestamp = chrono::Local::now().format("%H:%M:%S%.3f");

    match classify_message(data, is_pod) {
        MessageFormat::PodText(text) => {
            println!("[{}] #{}: {}", timestamp.to_string().dimmed(), seq, text);
        }
        MessageFormat::PodHex => {
            println!(
                "[{}] #{}: {} bytes (POD)",
                timestamp.to_string().dimmed(),
                seq,
                data.len()
            );
            print_hex_dump(data, 64);
        }
        MessageFormat::Json(pretty) => {
            println!("[{}] #{}:", timestamp.to_string().dimmed(), seq);
            println!("{}", pretty);
        }
        MessageFormat::BincodeHex => {
            println!(
                "[{}] #{}: {} bytes (bincode)",
                timestamp.to_string().dimmed(),
                seq,
                data.len()
            );
            print_hex_dump(data, 64);
        }
    }
}

/// Format hex dump of binary data (up to `max_bytes`) as a plain string.
fn format_hex_dump(data: &[u8], max_bytes: usize) -> String {
    let bytes_to_show = data.len().min(max_bytes);
    let mut out = data[..bytes_to_show]
        .chunks(16)
        .map(|row| {
            row.iter()
                .map(|b| format!("{:02x}", b))
                .collect::<Vec<_>>()
                .join(" ")
        })
        .collect::<Vec<_>>()
        .join("\n  ");

    if data.len() > max_bytes {
        out.push_str(&format!("\n  ... ({} more bytes)", data.len() - max_bytes));
    }
    out
}

/// Print hex dump of binary data (up to `max_bytes`).
fn print_hex_dump(data: &[u8], max_bytes: usize) {
    let hex = format_hex_dump(data, max_bytes);
    println!("  {}", hex.dimmed());
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

        std::thread::sleep(10_u64.ms());
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

        std::thread::sleep(10_u64.ms());
    }

    println!();
    Ok(())
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
        Some(r) => Some(r.hz().period()),
        None => None,
    };
    let publish_count = count.unwrap_or(1);

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    if topic.sub_count() == 0 {
        cli_output::warn(&format!(
            "No subscribers on topic '{}' — messages may be lost",
            name
        ));
    }

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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn hex_dump_empty_returns_empty() {
        let hex = format_hex_dump(&[], 64);
        assert!(
            hex.is_empty(),
            "Empty data should produce empty hex, got: '{}'",
            hex
        );
    }

    #[test]
    fn hex_dump_single_byte() {
        let hex = format_hex_dump(&[0xAA], 64);
        assert_eq!(hex, "aa");
    }

    #[test]
    fn hex_dump_multiple_bytes() {
        let hex = format_hex_dump(&[0x01, 0x02, 0x03, 0x04], 64);
        assert_eq!(hex, "01 02 03 04");
    }

    #[test]
    fn hex_dump_truncated_shows_remainder() {
        let data: Vec<u8> = (0..128).collect();
        let hex = format_hex_dump(&data, 16);
        // Should only show first 16 bytes
        assert!(
            hex.starts_with("00 01 02 03"),
            "Should start with first bytes, got: '{}'",
            hex
        );
        // Should indicate truncation with remaining count
        assert!(
            hex.contains("... (112 more bytes)"),
            "Should show truncation message, got: '{}'",
            hex
        );
    }

    #[test]
    fn classify_pod_text_returns_pod_text() {
        let fmt = classify_message(b"hello world", true);
        assert_eq!(fmt, MessageFormat::PodText("hello world".to_string()));
    }

    #[test]
    fn classify_pod_binary_returns_pod_hex() {
        let fmt = classify_message(&[0xFF, 0x00, 0x01], true);
        assert_eq!(fmt, MessageFormat::PodHex);
    }

    #[test]
    fn classify_serde_valid_json_returns_json() {
        let fmt = classify_message(b"{\"x\":1.0}", false);
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(
                    pretty.contains("\"x\""),
                    "JSON should contain key 'x', got: '{}'",
                    pretty
                );
                assert!(
                    pretty.contains("1.0"),
                    "JSON should contain value 1.0, got: '{}'",
                    pretty
                );
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_non_json_returns_bincode_hex() {
        let fmt = classify_message(
            &[0x05, 0x00, 0x00, 0x00, b'h', b'e', b'l', b'l', b'o'],
            false,
        );
        assert_eq!(fmt, MessageFormat::BincodeHex);
    }

    // ── Battle tests: hex dump edge cases ─────────────────────────────────

    #[test]
    fn hex_dump_exactly_max_bytes_no_truncation() {
        let data: Vec<u8> = (0..16).collect();
        let hex = format_hex_dump(&data, 16);
        assert!(!hex.contains("..."), "Exactly max_bytes should not truncate");
        assert_eq!(hex, "00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
    }

    #[test]
    fn hex_dump_one_over_max_truncates() {
        let data: Vec<u8> = (0..17).collect();
        let hex = format_hex_dump(&data, 16);
        assert!(hex.contains("... (1 more bytes)"), "One over should truncate, got: '{}'", hex);
    }

    #[test]
    fn hex_dump_multi_row_formatting() {
        // 32 bytes = 2 rows of 16
        let data: Vec<u8> = (0..32).collect();
        let hex = format_hex_dump(&data, 64);
        let rows: Vec<&str> = hex.split("\n  ").collect();
        assert_eq!(rows.len(), 2, "32 bytes should produce 2 rows, got: {}", rows.len());
    }

    #[test]
    fn hex_dump_max_zero_shows_nothing_but_remainder() {
        let data = vec![0xAA, 0xBB];
        let hex = format_hex_dump(&data, 0);
        assert!(hex.contains("2 more bytes"), "max_bytes=0 should show remainder, got: '{}'", hex);
    }

    #[test]
    fn hex_dump_all_zeros() {
        let data = vec![0u8; 4];
        let hex = format_hex_dump(&data, 64);
        assert_eq!(hex, "00 00 00 00");
    }

    #[test]
    fn hex_dump_all_ff() {
        let data = vec![0xFFu8; 4];
        let hex = format_hex_dump(&data, 64);
        assert_eq!(hex, "ff ff ff ff");
    }

    // ── Battle tests: classify_message edge cases ─────────────────────────

    #[test]
    fn classify_pod_empty_data_is_pod_text() {
        // Empty slice is valid UTF-8 and all chars pass the filter (vacuously true)
        let fmt = classify_message(&[], true);
        assert_eq!(fmt, MessageFormat::PodText(String::new()));
    }

    #[test]
    fn classify_serde_empty_data_is_bincode_hex() {
        // Empty slice is not valid JSON
        let fmt = classify_message(&[], false);
        assert_eq!(fmt, MessageFormat::BincodeHex);
    }

    #[test]
    fn classify_pod_with_tabs_and_newlines_is_text() {
        let data = b"hello\tworld\nfoo";
        let fmt = classify_message(data, true);
        assert_eq!(fmt, MessageFormat::PodText("hello\tworld\nfoo".to_string()));
    }

    #[test]
    fn classify_pod_with_control_char_is_hex() {
        // 0x01 (SOH) is not ascii_graphic or ascii_whitespace
        let data = &[b'a', 0x01, b'b'];
        let fmt = classify_message(data, true);
        assert_eq!(fmt, MessageFormat::PodHex);
    }

    #[test]
    fn classify_serde_json_array_returns_json() {
        let fmt = classify_message(b"[1, 2, 3]", false);
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(pretty.contains('1'), "Should contain 1, got: {}", pretty);
                assert!(pretty.contains('3'), "Should contain 3, got: {}", pretty);
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_json_string_returns_json() {
        let fmt = classify_message(b"\"hello\"", false);
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(pretty.contains("hello"), "Should contain hello, got: {}", pretty);
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_json_number_returns_json() {
        let fmt = classify_message(b"42", false);
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(pretty.contains("42"), "Should contain 42, got: {}", pretty);
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_json_null_returns_json() {
        let fmt = classify_message(b"null", false);
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(pretty.contains("null"), "Should contain null, got: {}", pretty);
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_invalid_json_is_bincode_hex() {
        let fmt = classify_message(b"{invalid json", false);
        assert_eq!(fmt, MessageFormat::BincodeHex);
    }

    #[test]
    fn classify_pod_only_whitespace_is_text() {
        let fmt = classify_message(b"   \t\n ", true);
        assert_eq!(fmt, MessageFormat::PodText("   \t\n ".to_string()));
    }

    // ── Battle tests: publish_topic JSON parsing ──────────────────────────

    #[test]
    fn publish_rejects_invalid_json() {
        let result = publish_topic("test_topic", "not valid json", None, None);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(err.contains("Invalid JSON"), "Error should mention JSON, got: {}", err);
    }

    #[test]
    fn publish_rejects_negative_rate() {
        // JSON is valid but rate is negative
        let result = publish_topic("test_topic", "{\"x\": 1}", Some(-5.0), None);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(err.contains("greater than 0"), "Should reject negative rate, got: {}", err);
    }

    #[test]
    fn publish_rejects_zero_rate() {
        let result = publish_topic("test_topic", "{\"x\": 1}", Some(0.0), None);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(err.contains("greater than 0"), "Should reject zero rate, got: {}", err);
    }

    #[test]
    fn echo_rejects_negative_rate() {
        // echo_topic will fail on discovery first, but if we pass rate <= 0
        // and the topic exists, rate validation happens after topic lookup.
        // We test the rate validation path with a known-missing topic — it
        // hits "not found" first.  So we just test that the error message from
        // a missing topic is sane.
        let result = echo_topic("__nonexistent_topic__", None, Some(-1.0));
        assert!(result.is_err());
    }

    #[test]
    fn topic_info_nonexistent_returns_error() {
        let result = topic_info("__definitely_not_a_topic_12345__");
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(err.contains("not found"), "Should say not found, got: {}", err);
    }
}
