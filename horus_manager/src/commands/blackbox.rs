//! `horus blackbox` / `horus bb` — read and inspect BlackBox flight recorder data.
//!
//! The BlackBox records critical scheduler events (errors, deadline misses,
//! budget violations, emergency stops) to a WAL file and JSON snapshot for
//! post-mortem crash analysis.

use anyhow::{Context, Result};
use colored::*;
use horus_core::core::DurationExt;
use horus_core::scheduling::{BlackBoxEvent, BlackBoxRecord};
use std::io::{BufRead, BufReader, Read, Seek, SeekFrom};
use std::path::{Path, PathBuf};

/// CLI arguments for the `horus blackbox` command.
pub struct BlackboxArgs {
    pub anomalies_only: bool,
    pub tail: bool,
    pub tick_range: Option<String>,
    pub node_filter: Option<String>,
    pub event_filter: Option<String>,
    pub json_output: bool,
    pub last_n: Option<usize>,
    pub custom_path: Option<PathBuf>,
    pub clear: bool,
}

/// Entry point for the `horus blackbox` command.
pub fn run_blackbox(args: BlackboxArgs) -> horus_core::error::HorusResult<()> {
    let BlackboxArgs {
        anomalies_only,
        tail,
        tick_range,
        node_filter,
        event_filter,
        json_output,
        last_n,
        custom_path,
        clear,
    } = args;
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
        let json = serde_json::to_string_pretty(&records).map_err(|e| {
            horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(
                e.to_string(),
            ))
        })?;
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
    crate::paths::blackbox_dir().map_err(|e| {
        horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(e.to_string()))
    })
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
        BlackBoxEvent::NodeAdded { name, order } => {
            format!("{} (order {})", name, order)
        }
        BlackBoxEvent::NodeTick {
            name,
            duration_us,
            success,
        } => {
            let status = if *success { "ok" } else { "FAIL" };
            format!("{} {}us [{}]", name, duration_us, status)
        }
        BlackBoxEvent::NodeError {
            name,
            error,
            severity,
        } => format!("{}: {} [{:?}]", name, error, severity),
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
        BlackBoxEvent::BudgetViolation {
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
        BlackBoxEvent::LearningComplete {
            duration_ms,
            tier_summary,
        } => format!("{}ms: {}", duration_ms, tier_summary),
        BlackBoxEvent::EmergencyStop { reason } => reason.clone(),
        BlackBoxEvent::NetPeerDiscovered {
            peer_addr,
            topic_count,
        } => {
            format!("Peer {peer_addr} discovered ({topic_count} topics)")
        }
        BlackBoxEvent::NetPeerLost { peer_addr, reason } => {
            format!("Peer {peer_addr} lost: {reason}")
        }
        BlackBoxEvent::NetReplicationStarted { peer_count } => {
            format!("Replication started ({peer_count} peers)")
        }
        BlackBoxEvent::NetImportRejected { topic, peer_addr } => {
            format!("Import rejected: '{topic}' from {peer_addr}")
        }
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
            std::thread::sleep(500_u64.ms());
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

    let mut file = std::fs::File::open(&wal_path).map_err(|e| {
        horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(e.to_string()))
    })?;
    file.seek(SeekFrom::End(0)).map_err(|e| {
        horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(e.to_string()))
    })?;

    let mut buf = String::new();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        let bytes_read = file.read_to_string(&mut buf).map_err(|e| {
            horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(
                e.to_string(),
            ))
        })?;

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
            std::thread::sleep(100_u64.ms());
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
        std::fs::remove_file(&wal_path).map_err(|e| {
            horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(
                e.to_string(),
            ))
        })?;
    }
    if json_exists {
        std::fs::remove_file(&json_path).map_err(|e| {
            horus_core::error::HorusError::Config(horus_core::error::ConfigError::Other(
                e.to_string(),
            ))
        })?;
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
        BlackBoxEvent::BudgetViolation { name, .. } => name,
        BlackBoxEvent::LearningComplete { .. } => "",
        BlackBoxEvent::EmergencyStop { .. } => "",
        BlackBoxEvent::NetPeerDiscovered { .. }
        | BlackBoxEvent::NetPeerLost { .. }
        | BlackBoxEvent::NetReplicationStarted { .. }
        | BlackBoxEvent::NetImportRejected { .. } => "",
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
        BlackBoxEvent::BudgetViolation { .. } => "BudgetViolation",
        BlackBoxEvent::LearningComplete { .. } => "LearningComplete",
        BlackBoxEvent::EmergencyStop { .. } => "EmergencyStop",
        BlackBoxEvent::NetPeerDiscovered { .. } => "NetPeerDiscovered",
        BlackBoxEvent::NetPeerLost { .. } => "NetPeerLost",
        BlackBoxEvent::NetReplicationStarted { .. } => "NetReplicationStarted",
        BlackBoxEvent::NetImportRejected { .. } => "NetImportRejected",
        BlackBoxEvent::Custom { .. } => "Custom",
    }
}

/// Whether an event is an anomaly (error, deadline miss, e-stop, etc.).
fn is_anomaly(event: &BlackBoxEvent) -> bool {
    matches!(
        event,
        BlackBoxEvent::NodeError { .. }
            | BlackBoxEvent::DeadlineMiss { .. }
            | BlackBoxEvent::BudgetViolation { .. }
            | BlackBoxEvent::EmergencyStop { .. }
            | BlackBoxEvent::NetPeerLost { .. }
            | BlackBoxEvent::NetImportRejected { .. }
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
        BlackBoxEvent::BudgetViolation { .. } => EventColor::Yellow,

        BlackBoxEvent::NodeError { .. }
        | BlackBoxEvent::DeadlineMiss { .. }
        | BlackBoxEvent::EmergencyStop { .. }
        | BlackBoxEvent::NetPeerLost { .. } => EventColor::Red,

        BlackBoxEvent::NetImportRejected { .. } => EventColor::Yellow,
        BlackBoxEvent::NetPeerDiscovered { .. } | BlackBoxEvent::NetReplicationStarted { .. } => {
            EventColor::Green
        }

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

#[cfg(test)]
mod tests {
    use super::*;

    // ── TickRange parsing ────────────────────────────────────────────────

    #[test]
    fn tick_range_single_value() {
        let r = TickRange::parse("4500").unwrap();
        assert_eq!(r.start, 4500);
        assert_eq!(r.end, 4500);
    }

    #[test]
    fn tick_range_range() {
        let r = TickRange::parse("100-200").unwrap();
        assert_eq!(r.start, 100);
        assert_eq!(r.end, 200);
    }

    #[test]
    fn tick_range_with_spaces() {
        let r = TickRange::parse(" 10 - 20 ").unwrap();
        assert_eq!(r.start, 10);
        assert_eq!(r.end, 20);
    }

    #[test]
    fn tick_range_invalid_format() {
        assert!(TickRange::parse("1-2-3").is_err());
    }

    #[test]
    fn tick_range_non_numeric() {
        assert!(TickRange::parse("abc").is_err());
    }

    // ── event_type_name ──────────────────────────────────────────────────

    #[test]
    fn event_type_names_correct() {
        assert_eq!(
            event_type_name(&BlackBoxEvent::SchedulerStart {
                name: "s".into(),
                node_count: 0,
                config: "c".into()
            }),
            "SchedulerStart"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::NodeError {
                name: "n".into(),
                error: "e".into(),
                severity: horus_core::error::Severity::Permanent,
            }),
            "NodeError"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::EmergencyStop { reason: "r".into() }),
            "EmergencyStop"
        );
    }

    // ── is_anomaly ───────────────────────────────────────────────────────

    #[test]
    fn anomaly_detection() {
        assert!(is_anomaly(&BlackBoxEvent::NodeError {
            name: "n".into(),
            error: "e".into(),
            severity: horus_core::error::Severity::Permanent,
        }));
        assert!(is_anomaly(&BlackBoxEvent::DeadlineMiss {
            name: "n".into(),
            deadline_us: 100,
            actual_us: 200
        }));
        assert!(is_anomaly(&BlackBoxEvent::EmergencyStop {
            reason: "r".into()
        }));
        // Non-anomalies
        assert!(!is_anomaly(&BlackBoxEvent::SchedulerStart {
            name: "s".into(),
            node_count: 1,
            config: "c".into()
        }));
        assert!(!is_anomaly(&BlackBoxEvent::NodeTick {
            name: "n".into(),
            duration_us: 50,
            success: true
        }));
    }

    // ── event_node_name ──────────────────────────────────────────────────

    #[test]
    fn event_node_name_extracts_correctly() {
        assert_eq!(
            event_node_name(&BlackBoxEvent::NodeTick {
                name: "motor_ctrl".into(),
                duration_us: 10,
                success: true
            }),
            "motor_ctrl"
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::SchedulerStop {
                reason: "done".into(),
                total_ticks: 100
            }),
            ""
        );
    }

    // ── record_matches filtering ─────────────────────────────────────────

    #[test]
    fn record_matches_no_filters() {
        let record = BlackBoxRecord {
            tick: 100,
            timestamp_us: 1234567890,
            event: BlackBoxEvent::NodeTick {
                name: "sensor".into(),
                duration_us: 50,
                success: true,
            },
        };
        assert!(record_matches(&record, false, &None, &None, &None));
    }

    #[test]
    fn record_matches_anomaly_filter() {
        let normal = BlackBoxRecord {
            tick: 1,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeTick {
                name: "n".into(),
                duration_us: 10,
                success: true,
            },
        };
        let anomaly = BlackBoxRecord {
            tick: 1,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeError {
                name: "n".into(),
                error: "crash".into(),
                severity: horus_core::error::Severity::Fatal,
            },
        };
        assert!(!record_matches(&normal, true, &None, &None, &None));
        assert!(record_matches(&anomaly, true, &None, &None, &None));
    }

    #[test]
    fn record_matches_tick_range_filter() {
        let record = BlackBoxRecord {
            tick: 50,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeTick {
                name: "n".into(),
                duration_us: 10,
                success: true,
            },
        };
        let in_range = Some(TickRange { start: 40, end: 60 });
        let out_of_range = Some(TickRange { start: 60, end: 80 });
        assert!(record_matches(&record, false, &in_range, &None, &None));
        assert!(!record_matches(&record, false, &out_of_range, &None, &None));
    }

    #[test]
    fn record_matches_node_filter() {
        let record = BlackBoxRecord {
            tick: 1,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeTick {
                name: "motor_controller".into(),
                duration_us: 10,
                success: true,
            },
        };
        let matching = Some("motor".to_string());
        let non_matching = Some("sensor".to_string());
        assert!(record_matches(&record, false, &None, &matching, &None));
        assert!(!record_matches(&record, false, &None, &non_matching, &None));
    }

    #[test]
    fn record_matches_event_filter() {
        let record = BlackBoxRecord {
            tick: 1,
            timestamp_us: 0,
            event: BlackBoxEvent::DeadlineMiss {
                name: "n".into(),
                deadline_us: 100,
                actual_us: 200,
            },
        };
        let matching = Some("DeadlineMiss".to_string());
        let non_matching = Some("NodeError".to_string());
        assert!(record_matches(&record, false, &None, &None, &matching));
        assert!(!record_matches(&record, false, &None, &None, &non_matching));
    }

    // ── format_timestamp ─────────────────────────────────────────────────

    #[test]
    fn format_timestamp_produces_string() {
        let ts = format_timestamp(1_000_000); // 1 second
                                              // Should be a formatted time, not empty
        assert!(!ts.is_empty());
    }

    #[test]
    fn format_timestamp_zero() {
        let ts = format_timestamp(0);
        assert!(!ts.is_empty());
    }

    // ── format_event_detail ──────────────────────────────────────────────

    #[test]
    fn format_event_detail_scheduler_start() {
        let detail = format_event_detail(&BlackBoxEvent::SchedulerStart {
            name: "main".into(),
            node_count: 5,
            config: "rt".into(),
        });
        assert!(detail.contains("main"));
        assert!(detail.contains("5 nodes"));
    }

    #[test]
    fn format_event_detail_deadline_miss() {
        let detail = format_event_detail(&BlackBoxEvent::DeadlineMiss {
            name: "ctrl".into(),
            deadline_us: 1000,
            actual_us: 1500,
        });
        assert!(detail.contains("ctrl"));
        assert!(detail.contains("1000"));
        assert!(detail.contains("1500"));
        assert!(detail.contains("+500")); // overshoot
    }

    // ── Battle tests: TickRange parsing edge cases ────────────────────────

    #[test]
    fn tick_range_zero() {
        let r = TickRange::parse("0").unwrap();
        assert_eq!(r.start, 0);
        assert_eq!(r.end, 0);
    }

    #[test]
    fn tick_range_max_u64() {
        let s = u64::MAX.to_string();
        let r = TickRange::parse(&s).unwrap();
        assert_eq!(r.start, u64::MAX);
        assert_eq!(r.end, u64::MAX);
    }

    #[test]
    fn tick_range_reversed_still_parses() {
        // 200-100 is technically valid (start > end), just won't match anything
        let r = TickRange::parse("200-100").unwrap();
        assert_eq!(r.start, 200);
        assert_eq!(r.end, 100);
    }

    #[test]
    fn tick_range_empty_string_fails() {
        assert!(TickRange::parse("").is_err());
    }

    #[test]
    fn tick_range_negative_fails() {
        assert!(TickRange::parse("-5").is_err());
    }

    #[test]
    fn tick_range_float_fails() {
        assert!(TickRange::parse("1.5").is_err());
    }

    #[test]
    fn tick_range_too_many_dashes() {
        assert!(TickRange::parse("1-2-3-4").is_err());
    }

    #[test]
    fn tick_range_whitespace_only_fails() {
        assert!(TickRange::parse("   ").is_err());
    }

    // ── Battle tests: event_type_name exhaustive ──────────────────────────

    #[test]
    fn event_type_name_all_variants() {
        assert_eq!(
            event_type_name(&BlackBoxEvent::SchedulerStop {
                reason: "".into(),
                total_ticks: 0
            }),
            "SchedulerStop"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::NodeAdded {
                name: "".into(),
                order: 0
            }),
            "NodeAdded"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::NodeTick {
                name: "".into(),
                duration_us: 0,
                success: true
            }),
            "NodeTick"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::BudgetViolation {
                name: "".into(),
                budget_us: 0,
                actual_us: 0
            }),
            "BudgetViolation"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::LearningComplete {
                duration_ms: 0,
                tier_summary: "".into()
            }),
            "LearningComplete"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::Custom {
                category: "".into(),
                message: "".into()
            }),
            "Custom"
        );
        assert_eq!(
            event_type_name(&BlackBoxEvent::DeadlineMiss {
                name: "".into(),
                deadline_us: 0,
                actual_us: 0
            }),
            "DeadlineMiss"
        );
    }

    // ── Battle tests: is_anomaly exhaustive ───────────────────────────────

    #[test]
    fn is_anomaly_budget_violation() {
        assert!(is_anomaly(&BlackBoxEvent::BudgetViolation {
            name: "node".into(),
            budget_us: 100,
            actual_us: 200,
        }));
    }

    #[test]
    fn is_anomaly_non_anomaly_variants() {
        assert!(!is_anomaly(&BlackBoxEvent::NodeAdded {
            name: "n".into(),
            order: 0
        }));
        assert!(!is_anomaly(&BlackBoxEvent::SchedulerStop {
            reason: "done".into(),
            total_ticks: 0
        }));
        assert!(!is_anomaly(&BlackBoxEvent::LearningComplete {
            duration_ms: 100,
            tier_summary: "ok".into()
        }));
        assert!(!is_anomaly(&BlackBoxEvent::Custom {
            category: "test".into(),
            message: "msg".into()
        }));
    }

    // ── Battle tests: event_node_name exhaustive ──────────────────────────

    #[test]
    fn event_node_name_all_empty_name_variants() {
        // Variants that return empty string for node name
        assert_eq!(
            event_node_name(&BlackBoxEvent::LearningComplete {
                duration_ms: 0,
                tier_summary: "".into()
            }),
            ""
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::EmergencyStop { reason: "".into() }),
            ""
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::Custom {
                category: "".into(),
                message: "".into()
            }),
            ""
        );
    }

    #[test]
    fn event_node_name_all_named_variants() {
        assert_eq!(
            event_node_name(&BlackBoxEvent::SchedulerStart {
                name: "sched".into(),
                node_count: 0,
                config: "".into()
            }),
            "sched"
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::NodeAdded {
                name: "added".into(),
                order: 0
            }),
            "added"
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::NodeError {
                name: "err_node".into(),
                error: "".into(),
                severity: horus_core::error::Severity::Transient
            }),
            "err_node"
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::DeadlineMiss {
                name: "miss".into(),
                deadline_us: 0,
                actual_us: 0
            }),
            "miss"
        );
        assert_eq!(
            event_node_name(&BlackBoxEvent::BudgetViolation {
                name: "budget".into(),
                budget_us: 0,
                actual_us: 0
            }),
            "budget"
        );
    }

    // ── Battle tests: record_matches combined filters ─────────────────────

    #[test]
    fn record_matches_combined_anomaly_and_node_filter() {
        let anomaly_motor = BlackBoxRecord {
            tick: 10,
            timestamp_us: 0,
            event: BlackBoxEvent::DeadlineMiss {
                name: "motor_ctrl".into(),
                deadline_us: 100,
                actual_us: 200,
            },
        };
        let anomaly_sensor = BlackBoxRecord {
            tick: 10,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeError {
                name: "sensor".into(),
                error: "timeout".into(),
                severity: horus_core::error::Severity::Transient,
            },
        };
        let normal_motor = BlackBoxRecord {
            tick: 10,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeTick {
                name: "motor_ctrl".into(),
                duration_us: 50,
                success: true,
            },
        };
        let motor_filter = Some("motor".to_string());
        // anomaly + motor filter: only anomaly_motor matches
        assert!(record_matches(
            &anomaly_motor,
            true,
            &None,
            &motor_filter,
            &None
        ));
        assert!(!record_matches(
            &anomaly_sensor,
            true,
            &None,
            &motor_filter,
            &None
        ));
        assert!(!record_matches(
            &normal_motor,
            true,
            &None,
            &motor_filter,
            &None
        ));
    }

    #[test]
    fn record_matches_tick_range_boundary() {
        let make_record = |tick: u64| BlackBoxRecord {
            tick,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeTick {
                name: "n".into(),
                duration_us: 10,
                success: true,
            },
        };
        let range = Some(TickRange {
            start: 100,
            end: 200,
        });
        // Boundary values
        assert!(record_matches(
            &make_record(100),
            false,
            &range,
            &None,
            &None
        )); // inclusive start
        assert!(record_matches(
            &make_record(200),
            false,
            &range,
            &None,
            &None
        )); // inclusive end
        assert!(record_matches(
            &make_record(150),
            false,
            &range,
            &None,
            &None
        )); // middle
        assert!(!record_matches(
            &make_record(99),
            false,
            &range,
            &None,
            &None
        )); // just below
        assert!(!record_matches(
            &make_record(201),
            false,
            &range,
            &None,
            &None
        )); // just above
    }

    #[test]
    fn record_matches_node_filter_case_insensitive() {
        let record = BlackBoxRecord {
            tick: 1,
            timestamp_us: 0,
            event: BlackBoxEvent::NodeTick {
                name: "MotorController".into(),
                duration_us: 10,
                success: true,
            },
        };
        assert!(record_matches(
            &record,
            false,
            &None,
            &Some("motor".to_string()),
            &None
        ));
        assert!(record_matches(
            &record,
            false,
            &None,
            &Some("MOTOR".to_string()),
            &None
        ));
        assert!(record_matches(
            &record,
            false,
            &None,
            &Some("controller".to_string()),
            &None
        ));
    }

    #[test]
    fn record_matches_event_filter_case_insensitive() {
        let record = BlackBoxRecord {
            tick: 1,
            timestamp_us: 0,
            event: BlackBoxEvent::EmergencyStop {
                reason: "test".into(),
            },
        };
        assert!(record_matches(
            &record,
            false,
            &None,
            &None,
            &Some("EmergencyStop".to_string())
        ));
        assert!(record_matches(
            &record,
            false,
            &None,
            &None,
            &Some("emergencystop".to_string())
        ));
        assert!(record_matches(
            &record,
            false,
            &None,
            &None,
            &Some("EMERGENCYSTOP".to_string())
        ));
        assert!(!record_matches(
            &record,
            false,
            &None,
            &None,
            &Some("NodeError".to_string())
        ));
    }

    // ── Battle tests: format_event_detail all variants ────────────────────

    #[test]
    fn format_event_detail_scheduler_stop() {
        let detail = format_event_detail(&BlackBoxEvent::SchedulerStop {
            reason: "shutdown".into(),
            total_ticks: 42000,
        });
        assert!(detail.contains("shutdown"));
        assert!(detail.contains("42000"));
    }

    #[test]
    fn format_event_detail_node_added() {
        let detail = format_event_detail(&BlackBoxEvent::NodeAdded {
            name: "lidar_proc".into(),
            order: 3,
        });
        assert!(detail.contains("lidar_proc"));
        assert!(detail.contains("3"));
    }

    #[test]
    fn format_event_detail_node_tick_success() {
        let detail = format_event_detail(&BlackBoxEvent::NodeTick {
            name: "ctrl".into(),
            duration_us: 250,
            success: true,
        });
        assert!(detail.contains("ctrl"));
        assert!(detail.contains("250"));
        assert!(detail.contains("ok"));
    }

    #[test]
    fn format_event_detail_node_tick_failure() {
        let detail = format_event_detail(&BlackBoxEvent::NodeTick {
            name: "ctrl".into(),
            duration_us: 9999,
            success: false,
        });
        assert!(detail.contains("FAIL"));
    }

    #[test]
    fn format_event_detail_node_error() {
        let detail = format_event_detail(&BlackBoxEvent::NodeError {
            name: "sensor".into(),
            error: "I2C timeout".into(),
            severity: horus_core::error::Severity::Transient,
        });
        assert!(detail.contains("sensor"));
        assert!(detail.contains("I2C timeout"));
        assert!(detail.contains("Transient"));
    }

    #[test]
    fn format_event_detail_budget_violation() {
        let detail = format_event_detail(&BlackBoxEvent::BudgetViolation {
            name: "planner".into(),
            budget_us: 500,
            actual_us: 800,
        });
        assert!(detail.contains("planner"));
        assert!(detail.contains("500"));
        assert!(detail.contains("800"));
        assert!(detail.contains("+300"));
    }

    #[test]
    fn format_event_detail_budget_violation_zero_overshoot() {
        let detail = format_event_detail(&BlackBoxEvent::BudgetViolation {
            name: "n".into(),
            budget_us: 100,
            actual_us: 100,
        });
        assert!(detail.contains("+0"));
    }

    #[test]
    fn format_event_detail_learning_complete() {
        let detail = format_event_detail(&BlackBoxEvent::LearningComplete {
            duration_ms: 5000,
            tier_summary: "tier1=3, tier2=2".into(),
        });
        assert!(detail.contains("5000"));
        assert!(detail.contains("tier1=3"));
    }

    #[test]
    fn format_event_detail_emergency_stop() {
        let detail = format_event_detail(&BlackBoxEvent::EmergencyStop {
            reason: "watchdog expired".into(),
        });
        assert_eq!(detail, "watchdog expired");
    }

    #[test]
    fn format_event_detail_custom() {
        let detail = format_event_detail(&BlackBoxEvent::Custom {
            category: "safety".into(),
            message: "proximity alert".into(),
        });
        assert!(detail.contains("[safety]"));
        assert!(detail.contains("proximity alert"));
    }

    // ── Battle tests: event_color ─────────────────────────────────────────

    #[test]
    fn event_color_green_for_lifecycle_events() {
        assert!(matches!(
            event_color(&BlackBoxEvent::SchedulerStart {
                name: "".into(),
                node_count: 0,
                config: "".into()
            }),
            EventColor::Green
        ));
        assert!(matches!(
            event_color(&BlackBoxEvent::SchedulerStop {
                reason: "".into(),
                total_ticks: 0
            }),
            EventColor::Green
        ));
        assert!(matches!(
            event_color(&BlackBoxEvent::NodeAdded {
                name: "".into(),
                order: 0
            }),
            EventColor::Green
        ));
        assert!(matches!(
            event_color(&BlackBoxEvent::LearningComplete {
                duration_ms: 0,
                tier_summary: "".into()
            }),
            EventColor::Green
        ));
    }

    #[test]
    fn event_color_red_for_critical_events() {
        assert!(matches!(
            event_color(&BlackBoxEvent::NodeError {
                name: "".into(),
                error: "".into(),
                severity: horus_core::error::Severity::Fatal
            }),
            EventColor::Red
        ));
        assert!(matches!(
            event_color(&BlackBoxEvent::DeadlineMiss {
                name: "".into(),
                deadline_us: 0,
                actual_us: 0
            }),
            EventColor::Red
        ));
        assert!(matches!(
            event_color(&BlackBoxEvent::EmergencyStop { reason: "".into() }),
            EventColor::Red
        ));
    }

    #[test]
    fn event_color_yellow_for_warnings() {
        assert!(matches!(
            event_color(&BlackBoxEvent::BudgetViolation {
                name: "".into(),
                budget_us: 0,
                actual_us: 0
            }),
            EventColor::Yellow
        ));
        assert!(matches!(
            event_color(&BlackBoxEvent::NodeTick {
                name: "".into(),
                duration_us: 0,
                success: false
            }),
            EventColor::Yellow
        ));
    }

    #[test]
    fn event_color_white_for_normal_tick() {
        assert!(matches!(
            event_color(&BlackBoxEvent::NodeTick {
                name: "".into(),
                duration_us: 0,
                success: true
            }),
            EventColor::White
        ));
    }

    // ── Battle tests: format_timestamp edge cases ─────────────────────────

    #[test]
    fn format_timestamp_large_value() {
        let ts = format_timestamp(1_700_000_000_000_000); // ~2023 epoch in us
                                                          // Should produce a human-readable time, not "us" fallback
        assert!(!ts.is_empty());
        assert!(ts.contains(':'), "should have HH:MM:SS format: {}", ts);
    }

    #[test]
    fn format_timestamp_max_u64_does_not_panic() {
        let ts = format_timestamp(u64::MAX);
        assert!(!ts.is_empty());
    }

    // ── Battle tests: load_wal with tempfile ──────────────────────────────

    #[test]
    fn load_wal_empty_file() {
        let dir = tempfile::tempdir().unwrap();
        let wal_path = dir.path().join("blackbox.wal");
        std::fs::write(&wal_path, "").unwrap();
        let records = load_wal(&wal_path).unwrap();
        assert!(records.is_empty());
    }

    #[test]
    fn load_wal_valid_jsonl() {
        let dir = tempfile::tempdir().unwrap();
        let wal_path = dir.path().join("blackbox.wal");
        let line1 = serde_json::to_string(&BlackBoxRecord {
            tick: 1,
            timestamp_us: 1000,
            event: BlackBoxEvent::NodeTick {
                name: "sensor".into(),
                duration_us: 50,
                success: true,
            },
        })
        .unwrap();
        let line2 = serde_json::to_string(&BlackBoxRecord {
            tick: 2,
            timestamp_us: 2000,
            event: BlackBoxEvent::EmergencyStop {
                reason: "watchdog".into(),
            },
        })
        .unwrap();
        std::fs::write(&wal_path, format!("{}\n{}\n", line1, line2)).unwrap();
        let records = load_wal(&wal_path).unwrap();
        assert_eq!(records.len(), 2);
        assert_eq!(records[0].tick, 1);
        assert_eq!(records[1].tick, 2);
    }

    #[test]
    fn load_wal_skips_corrupt_lines() {
        let dir = tempfile::tempdir().unwrap();
        let wal_path = dir.path().join("blackbox.wal");
        let valid = serde_json::to_string(&BlackBoxRecord {
            tick: 5,
            timestamp_us: 5000,
            event: BlackBoxEvent::NodeAdded {
                name: "ok".into(),
                order: 1,
            },
        })
        .unwrap();
        let content = format!("not valid json\n{}\n{{broken\n\n", valid);
        std::fs::write(&wal_path, content).unwrap();
        let records = load_wal(&wal_path).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].tick, 5);
    }

    #[test]
    fn load_wal_nonexistent_file_errors() {
        let result = load_wal(Path::new("/nonexistent/blackbox.wal"));
        result.unwrap_err();
    }

    #[test]
    fn load_blackbox_records_empty_dir() {
        let dir = tempfile::tempdir().unwrap();
        let records = load_blackbox_records(dir.path()).unwrap();
        assert!(records.is_empty());
    }

    #[test]
    fn load_blackbox_records_prefers_wal() {
        let dir = tempfile::tempdir().unwrap();
        // Write JSON with 1 record
        let json_records = vec![BlackBoxRecord {
            tick: 1,
            timestamp_us: 1000,
            event: BlackBoxEvent::NodeTick {
                name: "json".into(),
                duration_us: 10,
                success: true,
            },
        }];
        std::fs::write(
            dir.path().join("blackbox.json"),
            serde_json::to_string(&json_records).unwrap(),
        )
        .unwrap();
        // Write WAL with 2 records
        let wal_line1 = serde_json::to_string(&BlackBoxRecord {
            tick: 10,
            timestamp_us: 10000,
            event: BlackBoxEvent::NodeTick {
                name: "wal1".into(),
                duration_us: 20,
                success: true,
            },
        })
        .unwrap();
        let wal_line2 = serde_json::to_string(&BlackBoxRecord {
            tick: 11,
            timestamp_us: 11000,
            event: BlackBoxEvent::NodeTick {
                name: "wal2".into(),
                duration_us: 30,
                success: true,
            },
        })
        .unwrap();
        std::fs::write(
            dir.path().join("blackbox.wal"),
            format!("{}\n{}\n", wal_line1, wal_line2),
        )
        .unwrap();
        let records = load_blackbox_records(dir.path()).unwrap();
        assert_eq!(records.len(), 2, "should prefer WAL over JSON");
        assert_eq!(records[0].tick, 10);
    }

    #[test]
    fn load_blackbox_records_falls_back_to_json() {
        let dir = tempfile::tempdir().unwrap();
        let json_records = vec![BlackBoxRecord {
            tick: 42,
            timestamp_us: 42000,
            event: BlackBoxEvent::EmergencyStop {
                reason: "test".into(),
            },
        }];
        std::fs::write(
            dir.path().join("blackbox.json"),
            serde_json::to_string(&json_records).unwrap(),
        )
        .unwrap();
        let records = load_blackbox_records(dir.path()).unwrap();
        assert_eq!(records.len(), 1);
        assert_eq!(records[0].tick, 42);
    }

    // ── Battle tests: format_record does not panic ────────────────────────

    #[test]
    fn format_record_all_event_types_no_panic() {
        let events = vec![
            BlackBoxEvent::SchedulerStart {
                name: "main".into(),
                node_count: 3,
                config: "rt".into(),
            },
            BlackBoxEvent::SchedulerStop {
                reason: "done".into(),
                total_ticks: 1000,
            },
            BlackBoxEvent::NodeAdded {
                name: "ctrl".into(),
                order: 1,
            },
            BlackBoxEvent::NodeTick {
                name: "ctrl".into(),
                duration_us: 100,
                success: true,
            },
            BlackBoxEvent::NodeTick {
                name: "ctrl".into(),
                duration_us: 99999,
                success: false,
            },
            BlackBoxEvent::NodeError {
                name: "n".into(),
                error: "fail".into(),
                severity: horus_core::error::Severity::Fatal,
            },
            BlackBoxEvent::DeadlineMiss {
                name: "n".into(),
                deadline_us: 100,
                actual_us: 200,
            },
            BlackBoxEvent::BudgetViolation {
                name: "n".into(),
                budget_us: 50,
                actual_us: 75,
            },
            BlackBoxEvent::LearningComplete {
                duration_ms: 2000,
                tier_summary: "ok".into(),
            },
            BlackBoxEvent::EmergencyStop {
                reason: "critical fault".into(),
            },
            BlackBoxEvent::Custom {
                category: "user".into(),
                message: "hello".into(),
            },
        ];
        for (i, event) in events.into_iter().enumerate() {
            let record = BlackBoxRecord {
                tick: i as u64,
                timestamp_us: i as u64 * 1000,
                event,
            };
            format_record(&record); // should not panic
        }
    }
}
