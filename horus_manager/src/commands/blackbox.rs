//! `horus blackbox` / `horus bb` — read and inspect BlackBox flight recorder data.
//!
//! The BlackBox records critical scheduler events (errors, deadline misses,
//! WCET violations, emergency stops) to a WAL file and JSON snapshot for
//! post-mortem crash analysis.

use anyhow::{Context, Result};
use colored::*;
use horus_core::scheduling::{BlackBoxEvent, BlackBoxRecord};
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// Entry point for the `horus blackbox` command.
#[allow(clippy::too_many_arguments)]
pub fn run_blackbox(
    anomalies_only: bool,
    tail: bool,
    tick_range: Option<String>,
    node_filter: Option<String>,
    event_filter: Option<String>,
    json_output: bool,
    last_n: Option<usize>,
    custom_path: Option<PathBuf>,
    clear: bool,
) -> horus_core::error::HorusResult<()> {
    let bb_dir = resolve_blackbox_dir(custom_path)?;

    if clear {
        return clear_blackbox(&bb_dir);
    }

    if tail {
        return blackbox_tail(
            &bb_dir,
            anomalies_only,
            node_filter,
            event_filter,
            json_output,
        );
    }

    let tick = tick_range.map(|s| TickRange::parse(&s)).transpose()?;

    let mut records = load_blackbox_records(&bb_dir)?;

    // Apply filters
    records.retain(|r| record_matches(r, anomalies_only, &tick, &node_filter, &event_filter));

    // Apply --last N
    if let Some(n) = last_n {
        if records.len() > n {
            records = records.split_off(records.len() - n);
        }
    }

    if records.is_empty() {
        if json_output {
            println!("[]");
        } else {
            println!(
                "{} No blackbox events found in {}",
                "INFO".cyan().bold(),
                bb_dir.display()
            );
        }
        return Ok(());
    }

    if json_output {
        let json = serde_json::to_string_pretty(&records)
            .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;
        println!("{}", json);
    } else {
        println!(
            "{} {} events from {}\n",
            "BLACKBOX".cyan().bold(),
            records.len(),
            bb_dir.display()
        );
        for record in &records {
            format_record(record);
        }
    }

    Ok(())
}

/// Resolve the blackbox directory from a custom path or the default location.
fn resolve_blackbox_dir(custom_path: Option<PathBuf>) -> horus_core::error::HorusResult<PathBuf> {
    if let Some(p) = custom_path {
        return Ok(p);
    }
    crate::paths::blackbox_dir().map_err(|e| horus_core::error::HorusError::Config(e.to_string()))
}

/// Load BlackBoxRecords from the WAL file (preferred) or JSON snapshot (fallback).
fn load_blackbox_records(dir: &Path) -> Result<Vec<BlackBoxRecord>> {
    let wal_path = dir.join("blackbox.wal");
    let json_path = dir.join("blackbox.json");

    // Prefer WAL — it has the most recent entries (including crash data)
    if wal_path.is_file() {
        return load_wal(&wal_path);
    }

    // Fallback to JSON snapshot
    if json_path.is_file() {
        let content = std::fs::read_to_string(&json_path)
            .with_context(|| format!("reading {}", json_path.display()))?;
        let records: Vec<BlackBoxRecord> = serde_json::from_str(&content)
            .with_context(|| format!("parsing {}", json_path.display()))?;
        return Ok(records);
    }

    Ok(Vec::new())
}

/// Parse a JSONL WAL file into records, skipping corrupt lines.
fn load_wal(path: &Path) -> Result<Vec<BlackBoxRecord>> {
    let file = std::fs::File::open(path).with_context(|| format!("opening {}", path.display()))?;
    let reader = BufReader::new(file);
    let mut records = Vec::new();

    for line in reader.lines() {
        let line = match line {
            Ok(l) => l,
            Err(_) => continue,
        };
        let trimmed = line.trim();
        if trimmed.is_empty() {
            continue;
        }
        match serde_json::from_str::<BlackBoxRecord>(trimmed) {
            Ok(record) => records.push(record),
            Err(e) => {
                log::debug!("Skipping corrupt WAL line: {}", e);
            }
        }
    }

    Ok(records)
}

/// Check if a record matches all active filters.
fn record_matches(
    record: &BlackBoxRecord,
    anomalies_only: bool,
    tick: &Option<TickRange>,
    node_filter: &Option<String>,
    event_filter: &Option<String>,
) -> bool {
    if anomalies_only && !is_anomaly(&record.event) {
        return false;
    }

    if let Some(ref range) = tick {
        if record.tick < range.start || record.tick > range.end {
            return false;
        }
    }

    if let Some(ref node) = node_filter {
        let name = event_node_name(&record.event);
        if !name.to_lowercase().contains(&node.to_lowercase()) {
            return false;
        }
    }

    if let Some(ref ev) = event_filter {
        let type_name = event_type_name(&record.event);
        if !type_name.eq_ignore_ascii_case(ev) {
            return false;
        }
    }

    true
}

/// Format and print a single record with color coding.
fn format_record(record: &BlackBoxRecord) {
    let ts = format_timestamp(record.timestamp_us);
    let tick = format!("T{}", record.tick);
    let color = event_color(&record.event);
    let type_name = event_type_name(&record.event);
    let detail = format_event_detail(&record.event);

    let line = format!("{} {:>8} {:20} {}", ts, tick, type_name, detail);
    match color {
        EventColor::Green => println!("  {}", line.green()),
        EventColor::Yellow => println!("  {}", line.yellow()),
        EventColor::Red => println!("  {}", line.red()),
        EventColor::White => println!("  {}", line),
    }
}

/// Format microsecond timestamp as human-readable local time.
fn format_timestamp(us: u64) -> String {
    use chrono::{Local, TimeZone};
    let secs = (us / 1_000_000) as i64;
    let nanos = ((us % 1_000_000) * 1_000) as u32;
    match Local.timestamp_opt(secs, nanos) {
        chrono::LocalResult::Single(dt) => dt.format("%H:%M:%S%.3f").to_string(),
        _ => format!("{}us", us),
    }
}

/// Format the detail string for a single event.
fn format_event_detail(event: &BlackBoxEvent) -> String {
    match event {
        BlackBoxEvent::SchedulerStart {
            name,
            node_count,
            config,
        } => format!("{} ({} nodes, {})", name, node_count, config),
        BlackBoxEvent::SchedulerStop {
            reason,
            total_ticks,
        } => format!("{} after {} ticks", reason, total_ticks),
        BlackBoxEvent::NodeAdded { name, priority } => {
            format!("{} (priority {})", name, priority)
        }
        BlackBoxEvent::NodeTick {
            name,
            duration_us,
            success,
        } => {
            let status = if *success { "ok" } else { "FAIL" };
            format!("{} {}us [{}]", name, duration_us, status)
        }
        BlackBoxEvent::NodeError { name, error } => format!("{}: {}", name, error),
        BlackBoxEvent::DeadlineMiss {
            name,
            deadline_us,
            actual_us,
        } => format!(
            "{}: deadline {}us, actual {}us (+{}us)",
            name,
            deadline_us,
            actual_us,
            actual_us.saturating_sub(*deadline_us)
        ),
        BlackBoxEvent::WCETViolation {
            name,
            budget_us,
            actual_us,
        } => format!(
            "{}: budget {}us, actual {}us (+{}us)",
            name,
            budget_us,
            actual_us,
            actual_us.saturating_sub(*budget_us)
        ),
        BlackBoxEvent::CircuitBreakerChange {
            name,
            new_state,
            failure_count,
        } => format!("{} -> {} ({} failures)", name, new_state, failure_count),
        BlackBoxEvent::LearningComplete {
            duration_ms,
            tier_summary,
        } => format!("{}ms: {}", duration_ms, tier_summary),
        BlackBoxEvent::EmergencyStop { reason } => reason.clone(),
        BlackBoxEvent::Custom { category, message } => format!("[{}] {}", category, message),
    }
}

/// Poll-based tail mode: seek to end of WAL, print new lines as they arrive.
fn blackbox_tail(
    dir: &Path,
    anomalies_only: bool,
    node_filter: Option<String>,
    event_filter: Option<String>,
    json_output: bool,
) -> horus_core::error::HorusResult<()> {
    let wal_path = dir.join("blackbox.wal");

    if !wal_path.is_file() {
        println!(
            "{} Waiting for WAL file at {} ...",
            "TAIL".cyan().bold(),
            wal_path.display()
        );
        // Wait for file to appear
        while !wal_path.is_file() {
            std::thread::sleep(std::time::Duration::from_millis(500));
        }
    }

    println!(
        "{} Following {} (Ctrl+C to stop)\n",
        "TAIL".cyan().bold(),
        wal_path.display()
    );

    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    let mut file = std::fs::File::open(&wal_path)
        .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;
    file.seek(SeekFrom::End(0))
        .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;

    let mut buf = String::new();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        let bytes_read = file
            .read_to_string(&mut buf)
            .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;

        if bytes_read > 0 {
            for line in buf.lines() {
                let trimmed = line.trim();
                if trimmed.is_empty() {
                    continue;
                }
                if let Ok(record) = serde_json::from_str::<BlackBoxRecord>(trimmed) {
                    if record_matches(&record, anomalies_only, &None, &node_filter, &event_filter) {
                        if json_output {
                            println!("{}", serde_json::to_string(&record).unwrap_or_default());
                        } else {
                            format_record(&record);
                        }
                    }
                }
            }
            buf.clear();
        } else {
            std::thread::sleep(std::time::Duration::from_millis(100));
        }
    }

    println!("\n{}", "Stopped.".dimmed());
    Ok(())
}

/// Clear blackbox data with confirmation.
fn clear_blackbox(dir: &Path) -> horus_core::error::HorusResult<()> {
    let wal_path = dir.join("blackbox.wal");
    let json_path = dir.join("blackbox.json");

    let wal_exists = wal_path.is_file();
    let json_exists = json_path.is_file();

    if !wal_exists && !json_exists {
        println!("{} No blackbox data to clear.", "INFO".cyan().bold());
        return Ok(());
    }

    // Confirmation prompt
    use std::io::Write;
    print!(
        "{} Clear all blackbox data in {}? [y/N] ",
        "WARN".yellow().bold(),
        dir.display()
    );
    std::io::stdout().flush().ok();
    let mut input = String::new();
    std::io::stdin().read_line(&mut input).ok();
    if !input.trim().eq_ignore_ascii_case("y") {
        println!("Cancelled.");
        return Ok(());
    }

    if wal_exists {
        std::fs::remove_file(&wal_path)
            .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;
    }
    if json_exists {
        std::fs::remove_file(&json_path)
            .map_err(|e| horus_core::error::HorusError::Config(e.to_string()))?;
    }

    println!("{} Blackbox data cleared.", "OK".green().bold());
    Ok(())
}

// ── Helpers ─────────────────────────────────────────────────────────────────

/// Extract the node name from an event (empty string for non-node events).
fn event_node_name(event: &BlackBoxEvent) -> &str {
    match event {
        BlackBoxEvent::SchedulerStart { name, .. } => name,
        BlackBoxEvent::SchedulerStop { .. } => "",
        BlackBoxEvent::NodeAdded { name, .. } => name,
        BlackBoxEvent::NodeTick { name, .. } => name,
        BlackBoxEvent::NodeError { name, .. } => name,
        BlackBoxEvent::DeadlineMiss { name, .. } => name,
        BlackBoxEvent::WCETViolation { name, .. } => name,
        BlackBoxEvent::CircuitBreakerChange { name, .. } => name,
        BlackBoxEvent::LearningComplete { .. } => "",
        BlackBoxEvent::EmergencyStop { .. } => "",
        BlackBoxEvent::Custom { .. } => "",
    }
}

/// Get a short type name for an event (used for --event filtering).
fn event_type_name(event: &BlackBoxEvent) -> &'static str {
    match event {
        BlackBoxEvent::SchedulerStart { .. } => "SchedulerStart",
        BlackBoxEvent::SchedulerStop { .. } => "SchedulerStop",
        BlackBoxEvent::NodeAdded { .. } => "NodeAdded",
        BlackBoxEvent::NodeTick { .. } => "NodeTick",
        BlackBoxEvent::NodeError { .. } => "NodeError",
        BlackBoxEvent::DeadlineMiss { .. } => "DeadlineMiss",
        BlackBoxEvent::WCETViolation { .. } => "WCETViolation",
        BlackBoxEvent::CircuitBreakerChange { .. } => "CircuitBreakerChange",
        BlackBoxEvent::LearningComplete { .. } => "LearningComplete",
        BlackBoxEvent::EmergencyStop { .. } => "EmergencyStop",
        BlackBoxEvent::Custom { .. } => "Custom",
    }
}

/// Whether an event is an anomaly (error, deadline miss, e-stop, etc.).
fn is_anomaly(event: &BlackBoxEvent) -> bool {
    matches!(
        event,
        BlackBoxEvent::NodeError { .. }
            | BlackBoxEvent::DeadlineMiss { .. }
            | BlackBoxEvent::WCETViolation { .. }
            | BlackBoxEvent::EmergencyStop { .. }
            | BlackBoxEvent::CircuitBreakerChange { .. }
    )
}

enum EventColor {
    Green,
    Yellow,
    Red,
    White,
}

/// Color category for an event.
fn event_color(event: &BlackBoxEvent) -> EventColor {
    match event {
        BlackBoxEvent::SchedulerStart { .. }
        | BlackBoxEvent::SchedulerStop { .. }
        | BlackBoxEvent::NodeAdded { .. }
        | BlackBoxEvent::LearningComplete { .. } => EventColor::Green,

        BlackBoxEvent::NodeTick { success, .. } if !success => EventColor::Yellow,
        BlackBoxEvent::CircuitBreakerChange { .. } | BlackBoxEvent::WCETViolation { .. } => {
            EventColor::Yellow
        }

        BlackBoxEvent::NodeError { .. }
        | BlackBoxEvent::DeadlineMiss { .. }
        | BlackBoxEvent::EmergencyStop { .. } => EventColor::Red,

        _ => EventColor::White,
    }
}

/// A parsed tick range (e.g. "4500-4510").
struct TickRange {
    start: u64,
    end: u64,
}

impl TickRange {
    fn parse(s: &str) -> Result<Self> {
        let parts: Vec<&str> = s.split('-').collect();
        match parts.len() {
            1 => {
                let tick: u64 = parts[0]
                    .trim()
                    .parse()
                    .with_context(|| format!("invalid tick: '{}'", s))?;
                Ok(TickRange {
                    start: tick,
                    end: tick,
                })
            }
            2 => {
                let start: u64 = parts[0]
                    .trim()
                    .parse()
                    .with_context(|| format!("invalid tick range start: '{}'", parts[0]))?;
                let end: u64 = parts[1]
                    .trim()
                    .parse()
                    .with_context(|| format!("invalid tick range end: '{}'", parts[1]))?;
                Ok(TickRange { start, end })
            }
            _ => anyhow::bail!("invalid tick range format: '{}' (expected N or N-M)", s),
        }
    }
}
