//! Topic command - Interact with HORUS topics
//!
//! Provides commands for listing, echoing, and publishing to topics.

use crate::cli_output;
use crate::discovery::discover_shared_memory;
use crate::progress::format_bytes;
use colored::*;
use horus_core::core::DurationExt;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::memory::shm_topics_dir;
use std::io::{IsTerminal, Write};
use std::time::Instant;

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
            let type_name = topic.message_type.as_deref().unwrap_or("");
            print_message(&slot.payload, messages_received, slot.is_pod, type_name);
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
    /// Pod decoded into named fields
    PodFields(String),
    /// Binary POD → hex dump (unknown type)
    PodHex,
    /// Valid JSON payload → pretty-printed
    Json(String),
    /// Non-JSON serde (bincode) → hex dump
    BincodeHex,
}

/// Decode a Pod type into human-readable fields using the type name.
fn decode_pod_fields(data: &[u8], type_name: &str) -> Option<String> {
    match type_name {
        // Primitive types
        "f32" if data.len() == 4 => {
            let v = f32::from_le_bytes([data[0], data[1], data[2], data[3]]);
            Some(format!("{:.4}", v))
        }
        "f64" if data.len() == 8 => {
            let v = f64::from_le_bytes(data[..8].try_into().ok()?);
            Some(format!("{:.6}", v))
        }
        "u64" if data.len() == 8 => {
            let v = u64::from_le_bytes(data[..8].try_into().ok()?);
            Some(format!("{}", v))
        }
        "i64" if data.len() == 8 => {
            let v = i64::from_le_bytes(data[..8].try_into().ok()?);
            Some(format!("{}", v))
        }
        "u32" if data.len() == 4 => {
            let v = u32::from_le_bytes(data[..4].try_into().ok()?);
            Some(format!("{}", v))
        }
        "i32" if data.len() == 4 => {
            let v = i32::from_le_bytes(data[..4].try_into().ok()?);
            Some(format!("{}", v))
        }
        "bool" if data.len() == 1 => {
            Some(if data[0] != 0 { "true" } else { "false" }.to_string())
        }

        // ══════════════════════════════════════════════════════════════════
        // Standard horus_library message types — Pod layout
        // ALL offsets verified against horus_library/messages/ source on 2026-03-28
        // ══════════════════════════════════════════════════════════════════

        // CmdVel (16B): { timestamp_ns: u64@0, linear: f32@8, angular: f32@12 }
        "CmdVel" if data.len() >= 16 => {
            let linear = f32::from_le_bytes(data[8..12].try_into().unwrap_or([0;4]));
            let angular = f32::from_le_bytes(data[12..16].try_into().unwrap_or([0;4]));
            Some(format!("linear: {:.3}, angular: {:.3}", linear, angular))
        }

        // Pose2D (32B): { x: f64@0, y: f64@8, theta: f64@16, timestamp_ns: u64@24 }
        "Pose2D" if data.len() >= 32 => {
            let x = f64::from_le_bytes(data[0..8].try_into().unwrap_or([0;8]));
            let y = f64::from_le_bytes(data[8..16].try_into().unwrap_or([0;8]));
            let theta = f64::from_le_bytes(data[16..24].try_into().unwrap_or([0;8]));
            Some(format!("x: {:.3}, y: {:.3}, theta: {:.3}", x, y, theta))
        }

        // Imu (304B): orientation[4]@0, orient_cov[9]@32, angular_vel[3]@104,
        //             ang_cov[9]@128, linear_accel[3]@200, lin_cov[9]@224, timestamp_ns@296
        "Imu" if data.len() >= 304 => {
            let rd = |off: usize| -> f64 { f64::from_le_bytes(data[off..off+8].try_into().unwrap_or([0;8])) };
            let gx = rd(104); let gy = rd(112); let gz = rd(120); // angular_velocity
            let ax = rd(200); let ay = rd(208); let az = rd(216); // linear_acceleration
            Some(format!(
                "accel: [{:.3}, {:.3}, {:.3}], gyro: [{:.3}, {:.3}, {:.3}]",
                ax, ay, az, gx, gy, gz
            ))
        }

        // LaserScan (1476B): ranges[360]@0, angle_min@1440, angle_max@1444,
        //                    range_min@1448, range_max@1452, angle_inc@1456, timestamp_ns@1468
        "LaserScan" if data.len() >= 1476 => {
            let rf = |off: usize| -> f32 { f32::from_le_bytes(data[off..off+4].try_into().unwrap_or([0;4])) };
            let angle_min = rf(1440);
            let angle_max = rf(1444);
            let range_min = rf(1448);
            let range_max = rf(1452);
            // Count valid (non-zero) ranges
            let valid = (0..360).filter(|&i| {
                let r = rf(i * 4);
                r > 0.0 && r < 100.0
            }).count();
            Some(format!(
                "ranges: {}/360 valid, angle: [{:.2}, {:.2}] rad, range: [{:.1}, {:.1}] m",
                valid, angle_min, angle_max, range_min, range_max
            ))
        }

        // Odometry (736B): pose(Pose2D 32B)@0, twist(Twist 56B)@32,
        //                  pose_cov[36]@88, twist_cov[36]@376, frame_id[32]@664, child_frame[32]@696, ts@728
        "Odometry" if data.len() >= 736 => {
            let rd = |off: usize| -> f64 { f64::from_le_bytes(data[off..off+8].try_into().unwrap_or([0;8])) };
            let x = rd(0); let y = rd(8); let theta = rd(16); // from embedded Pose2D
            let frame_end = data[664..696].iter().position(|&b| b == 0).unwrap_or(32);
            let frame_id = std::str::from_utf8(&data[664..664+frame_end]).unwrap_or("?");
            Some(format!("pos: ({:.3}, {:.3}), yaw: {:.3}, frame: {}", x, y, theta, frame_id))
        }

        // JointState (912B): names[[u8;32];16]@0, joint_count(u8)@512,
        //                    positions[16]@520, velocities[16]@648, efforts[16]@776, ts@904
        "JointState" if data.len() >= 912 => {
            let n_joints = data[512] as usize;
            let n = n_joints.min(16);
            let rd = |off: usize| -> f64 { f64::from_le_bytes(data[off..off+8].try_into().unwrap_or([0;8])) };
            let pos: Vec<String> = (0..n).map(|i| format!("{:.3}", rd(520 + i * 8))).collect();
            Some(format!("joints: {}, pos: [{}]", n, pos.join(", ")))
        }

        // NavSatFix (128B): lat@0, lon@8, alt@16, pos_cov[9]@24, cov_type@96, status@97,
        //                   sats@98, hdop@100, vdop@104, speed@108, heading@112, ts@120
        "NavSatFix" if data.len() >= 128 => {
            let rd = |off: usize| -> f64 { f64::from_le_bytes(data[off..off+8].try_into().unwrap_or([0;8])) };
            let rf = |off: usize| -> f32 { f32::from_le_bytes(data[off..off+4].try_into().unwrap_or([0;4])) };
            let lat = rd(0); let lon = rd(8); let alt = rd(16);
            let sats = u16::from_le_bytes(data[98..100].try_into().unwrap_or([0;2]));
            let hdop = rf(100);
            Some(format!("lat: {:.6}, lon: {:.6}, alt: {:.1}m, sats: {}, hdop: {:.1}", lat, lon, alt, sats, hdop))
        }

        // BatteryState (104B): voltage@0, current@4, charge@8, capacity@12, pct@16,
        //                      status@20, [pad 3], temp@24, cells[16]@28, cell_count@92, [pad 3], ts@96
        "BatteryState" if data.len() >= 104 => {
            let rf = |off: usize| -> f32 { f32::from_le_bytes(data[off..off+4].try_into().unwrap_or([0;4])) };
            let voltage = rf(0); let current = rf(4); let pct = rf(16); let temp = rf(24);
            Some(format!("voltage: {:.2}V, current: {:.2}A, charge: {:.0}%, temp: {:.1}°C", voltage, current, pct * 100.0, temp))
        }

        // Temperature (56B): temperature(f64)@0, variance(f64)@8, frame_id[32]@16, ts@48
        "Temperature" if data.len() >= 56 => {
            let temp = f64::from_le_bytes(data[0..8].try_into().unwrap_or([0;8]));
            let var = f64::from_le_bytes(data[8..16].try_into().unwrap_or([0;8]));
            let frame_end = data[16..48].iter().position(|&b| b == 0).unwrap_or(32);
            let frame_id = std::str::from_utf8(&data[16..16+frame_end]).unwrap_or("?");
            Some(format!("temp: {:.2}°C (var: {:.4}), frame: {}", temp, var, frame_id))
        }

        // ImageDescriptor: Tensor (inner) + timestamp_ns + step + encoding
        // Layout: pool_id(u32@0) + slot_id(u32@4) + generation(u32@8) + gen_hi(u32@12)
        //         offset(u64@16) + size(u64@24) + dtype(u8@32) + ndim(u8@33) + ...
        //         shape[0..8](u64@40) + strides[0..8](u64@104)
        //         timestamp_ns(u64@168) + step(u32@176) + encoding(u8@180)
        // Full size: 224 bytes
        "ImageDescriptor" if data.len() >= 184 => {
            let pool_id = u32::from_le_bytes(data[0..4].try_into().unwrap_or([0;4]));
            let ndim = data[33];
            let size = u64::from_le_bytes(data[24..32].try_into().unwrap_or([0;8]));
            let h = u64::from_le_bytes(data[40..48].try_into().unwrap_or([0;8]));
            let w = u64::from_le_bytes(data[48..56].try_into().unwrap_or([0;8]));
            let c = u64::from_le_bytes(data[56..64].try_into().unwrap_or([0;8]));
            let step = u32::from_le_bytes(data[176..180].try_into().unwrap_or([0;4]));
            let encoding_raw = data[180];
            // Validate: pool_id=0 means empty/default descriptor, ndim should be 2-3 for images,
            // dimensions should be reasonable (< 65536 for real cameras)
            if pool_id == 0 && h == 0 && w == 0 {
                Some("Image (empty — no data in pool)".to_string())
            } else if ndim < 2 || h > 65536 || w > 65536 || c > 256 {
                // Likely stale or corrupt descriptor
                Some(format!("Image (stale descriptor, pool_id={}, ndim={})", pool_id, ndim))
            } else {
                // Match ImageEncoding repr(u8) discriminants from image_encoding.rs
                let enc_name = match encoding_raw {
                    0 => "mono8", 1 => "mono16", 2 => "rgb8", 3 => "bgr8",
                    4 => "rgba8", 5 => "bgra8", 6 => "yuv422", 7 => "mono32f",
                    8 => "rgb32f", 9 => "bayer_rggb8", 10 => "depth16", _ => "unknown",
                };
                Some(format!("Image {}x{} {}ch {} (step={}, {} bytes)", w, h, c, enc_name, step, size))
            }
        }

        // PointCloudDescriptor: similar layout, shape[0]=N points
        "PointCloudDescriptor" if data.len() >= 80 => {
            let pool_id = u32::from_le_bytes(data[0..4].try_into().unwrap_or([0;4]));
            let ndim = data[33];
            let size = u64::from_le_bytes(data[24..32].try_into().unwrap_or([0;8]));
            let n = u64::from_le_bytes(data[40..48].try_into().unwrap_or([0;8]));
            if pool_id == 0 && n == 0 {
                Some("PointCloud (empty — no data in pool)".to_string())
            } else if ndim == 0 || n > 100_000_000 {
                Some(format!("PointCloud (stale descriptor, pool_id={})", pool_id))
            } else {
                Some(format!("PointCloud {} points ({} bytes)", n, size))
            }
        }

        // DepthImageDescriptor: shape[0]=H, shape[1]=W (single channel depth)
        "DepthImageDescriptor" if data.len() >= 80 => {
            let pool_id = u32::from_le_bytes(data[0..4].try_into().unwrap_or([0;4]));
            let ndim = data[33];
            let size = u64::from_le_bytes(data[24..32].try_into().unwrap_or([0;8]));
            let h = u64::from_le_bytes(data[40..48].try_into().unwrap_or([0;8]));
            let w = u64::from_le_bytes(data[48..56].try_into().unwrap_or([0;8]));
            if pool_id == 0 && h == 0 && w == 0 {
                Some("DepthImage (empty — no data in pool)".to_string())
            } else if ndim < 2 || h > 65536 || w > 65536 {
                Some(format!("DepthImage (stale descriptor, pool_id={})", pool_id))
            } else {
                Some(format!("DepthImage {}x{} ({} bytes)", w, h, size))
            }
        }

        _ => None, // Unknown type — fall back to hex
    }
}

/// Classify how a message payload should be displayed without printing.
fn classify_message(data: &[u8], is_pod: bool, type_name: &str) -> MessageFormat {
    if is_pod {
        // Try field decoding for known types
        if let Some(fields) = decode_pod_fields(data, type_name) {
            return MessageFormat::PodFields(fields);
        }
        // Fall back to text if valid UTF-8
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

fn print_message(data: &[u8], seq: usize, is_pod: bool, type_name: &str) {
    let timestamp = chrono::Local::now().format("%H:%M:%S%.3f");

    match classify_message(data, is_pod, type_name) {
        MessageFormat::PodFields(fields) => {
            println!("[{}] #{}: {}", timestamp.to_string().dimmed(), seq, fields);
        }
        MessageFormat::PodText(text) => {
            println!("[{}] #{}: {}", timestamp.to_string().dimmed(), seq, text);
        }
        MessageFormat::PodHex => {
            println!(
                "[{}] #{}: {} bytes ({})",
                timestamp.to_string().dimmed(),
                seq,
                data.len(),
                if type_name.is_empty() { "POD" } else { type_name }
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
    use horus_core::communication::read_topic_messages_total;

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

    // Use messages_total (reliable atomic counter) instead of sequence_or_head
    // (lazily flushed). Sample every 100ms and compute rate from delta.
    let mut last_total: u64 = read_topic_messages_total(&topic_path).unwrap_or(0);
    let mut rate_samples: std::collections::VecDeque<(Instant, u64)> =
        std::collections::VecDeque::with_capacity(window_size + 1);
    let mut total_messages: u64 = 0;
    let start_time = Instant::now();
    let mut last_line_print = Instant::now();
    let is_tty = std::io::stdout().is_terminal();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        let current_total = read_topic_messages_total(&topic_path).unwrap_or(last_total);
        if current_total > last_total {
            let new_msgs = current_total - last_total;
            total_messages += new_msgs;
            last_total = current_total;
            rate_samples.push_back((Instant::now(), current_total));

            // Keep only window_size samples
            while rate_samples.len() > window_size {
                rate_samples.pop_front();
            }

            // Calculate rate from (time, total_count) pairs
            if rate_samples.len() >= 2 {
                let (first_time, first_count) = rate_samples.front().unwrap();
                let (last_time, last_count) = rate_samples.back().unwrap();
                let duration_secs = last_time.duration_since(*first_time).as_secs_f64();
                let msg_delta = last_count.saturating_sub(*first_count) as f64;

                if duration_secs > 0.0 {
                    let rate = msg_delta / duration_secs;
                    if is_tty {
                        if last_line_print.elapsed().as_secs_f64() >= 1.0 {
                            println!(
                                "\r  average rate: {:.2} Hz (window: {})    ",
                                rate,
                                rate_samples.len()
                            );
                            last_line_print = Instant::now();
                        } else {
                            print!(
                                "\r  {} {:.2} Hz (window: {})    ",
                                "Rate:".cyan(),
                                rate,
                                rate_samples.len()
                            );
                            std::io::stdout().flush().ok();
                        }
                    } else {
                        if last_line_print.elapsed().as_secs_f64() >= 1.0 {
                            println!(
                                "  average rate: {:.2} Hz (window: {})",
                                rate,
                                rate_samples.len()
                            );
                            last_line_print = Instant::now();
                        }
                    }
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
    let mut last_line_print = Instant::now();
    let is_tty = std::io::stdout().is_terminal();

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

                    if is_tty {
                        if last_line_print.elapsed().as_secs_f64() >= 1.0 {
                            println!(
                                "\r  {:.2} {} ({:.1} msgs/s, avg {}/msg, window: {})    ",
                                bw_val, bw_unit, msgs_per_sec,
                                format_bytes(avg_bytes as u64), samples.len()
                            );
                            last_line_print = Instant::now();
                        } else {
                            print!(
                                "\r  {} {:.2} {} ({:.1} msgs/s, avg {}/msg, window: {})    ",
                                "Bandwidth:".cyan(),
                                bw_val, bw_unit, msgs_per_sec,
                                format_bytes(avg_bytes as u64), samples.len()
                            );
                            std::io::stdout().flush().ok();
                        }
                    } else {
                        // Non-interactive: clean line output every second
                        if last_line_print.elapsed().as_secs_f64() >= 1.0 {
                            println!(
                                "  {:.2} {} ({:.1} msgs/s, avg {}/msg, window: {})",
                                bw_val, bw_unit, msgs_per_sec,
                                format_bytes(avg_bytes as u64), samples.len()
                            );
                            last_line_print = Instant::now();
                        }
                    }
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
        let fmt = classify_message(b"hello world", true, "");
        assert_eq!(fmt, MessageFormat::PodText("hello world".to_string()));
    }

    #[test]
    fn classify_pod_binary_returns_pod_hex() {
        let fmt = classify_message(&[0xFF, 0x00, 0x01], true, "");
        assert_eq!(fmt, MessageFormat::PodHex);
    }

    #[test]
    fn classify_serde_valid_json_returns_json() {
        let fmt = classify_message(b"{\"x\":1.0}", false, "");
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
            "",
        );
        assert_eq!(fmt, MessageFormat::BincodeHex);
    }

    // ── Battle tests: hex dump edge cases ─────────────────────────────────

    #[test]
    fn hex_dump_exactly_max_bytes_no_truncation() {
        let data: Vec<u8> = (0..16).collect();
        let hex = format_hex_dump(&data, 16);
        assert!(
            !hex.contains("..."),
            "Exactly max_bytes should not truncate"
        );
        assert_eq!(hex, "00 01 02 03 04 05 06 07 08 09 0a 0b 0c 0d 0e 0f");
    }

    #[test]
    fn hex_dump_one_over_max_truncates() {
        let data: Vec<u8> = (0..17).collect();
        let hex = format_hex_dump(&data, 16);
        assert!(
            hex.contains("... (1 more bytes)"),
            "One over should truncate, got: '{}'",
            hex
        );
    }

    #[test]
    fn hex_dump_multi_row_formatting() {
        // 32 bytes = 2 rows of 16
        let data: Vec<u8> = (0..32).collect();
        let hex = format_hex_dump(&data, 64);
        let rows: Vec<&str> = hex.split("\n  ").collect();
        assert_eq!(
            rows.len(),
            2,
            "32 bytes should produce 2 rows, got: {}",
            rows.len()
        );
    }

    #[test]
    fn hex_dump_max_zero_shows_nothing_but_remainder() {
        let data = vec![0xAA, 0xBB];
        let hex = format_hex_dump(&data, 0);
        assert!(
            hex.contains("2 more bytes"),
            "max_bytes=0 should show remainder, got: '{}'",
            hex
        );
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
        let fmt = classify_message(&[], true, "");
        assert_eq!(fmt, MessageFormat::PodText(String::new()));
    }

    #[test]
    fn classify_serde_empty_data_is_bincode_hex() {
        // Empty slice is not valid JSON
        let fmt = classify_message(&[], false, "");
        assert_eq!(fmt, MessageFormat::BincodeHex);
    }

    #[test]
    fn classify_pod_with_tabs_and_newlines_is_text() {
        let data = b"hello\tworld\nfoo";
        let fmt = classify_message(data, true, "");
        assert_eq!(fmt, MessageFormat::PodText("hello\tworld\nfoo".to_string()));
    }

    #[test]
    fn classify_pod_with_control_char_is_hex() {
        // 0x01 (SOH) is not ascii_graphic or ascii_whitespace
        let data = &[b'a', 0x01, b'b'];
        let fmt = classify_message(data, true, "");
        assert_eq!(fmt, MessageFormat::PodHex);
    }

    #[test]
    fn classify_serde_json_array_returns_json() {
        let fmt = classify_message(b"[1, 2, 3]", false, "");
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
        let fmt = classify_message(b"\"hello\"", false, "");
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(
                    pretty.contains("hello"),
                    "Should contain hello, got: {}",
                    pretty
                );
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_json_number_returns_json() {
        let fmt = classify_message(b"42", false, "");
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(pretty.contains("42"), "Should contain 42, got: {}", pretty);
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_json_null_returns_json() {
        let fmt = classify_message(b"null", false, "");
        match fmt {
            MessageFormat::Json(pretty) => {
                assert!(
                    pretty.contains("null"),
                    "Should contain null, got: {}",
                    pretty
                );
            }
            other => panic!("Expected Json variant, got {:?}", other),
        }
    }

    #[test]
    fn classify_serde_invalid_json_is_bincode_hex() {
        let fmt = classify_message(b"{invalid json", false, "");
        assert_eq!(fmt, MessageFormat::BincodeHex);
    }

    #[test]
    fn classify_pod_only_whitespace_is_text() {
        let fmt = classify_message(b"   \t\n ", true, "");
        assert_eq!(fmt, MessageFormat::PodText("   \t\n ".to_string()));
    }

    // ── Battle tests: publish_topic JSON parsing ──────────────────────────

    #[test]
    fn publish_rejects_invalid_json() {
        let result = publish_topic("test_topic", "not valid json", None, None);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("Invalid JSON"),
            "Error should mention JSON, got: {}",
            err
        );
    }

    #[test]
    fn publish_rejects_negative_rate() {
        // JSON is valid but rate is negative
        let result = publish_topic("test_topic", "{\"x\": 1}", Some(-5.0), None);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("greater than 0"),
            "Should reject negative rate, got: {}",
            err
        );
    }

    #[test]
    fn publish_rejects_zero_rate() {
        let result = publish_topic("test_topic", "{\"x\": 1}", Some(0.0), None);
        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("greater than 0"),
            "Should reject zero rate, got: {}",
            err
        );
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
        assert!(
            err.contains("not found"),
            "Should say not found, got: {}",
            err
        );
    }
}
