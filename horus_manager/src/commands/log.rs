//! Log command - View and filter HORUS logs
//!
//! Reads from the shared-memory ring buffer (`GLOBAL_LOG_BUFFER`) written by
//! the `hlog!` macro and the `log::` bridge. This is the same source that the
//! TUI monitor and web monitor use, so the output is always consistent.

use crate::cli_output;
use colored::*;
use horus_core::core::log_buffer::{LogEntry, LogType, GLOBAL_LOG_BUFFER};
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::memory::shm_logs_path;
use std::time::SystemTime;
use horus_core::core::DurationExt;

// ─── Log level for filtering ──────────────────────────────────────────────────

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub enum LogLevel {
    Trace,
    Debug,
    Info,
    Warn,
    Error,
}

impl LogLevel {
    fn from_str(s: &str) -> Option<Self> {
        match s.to_lowercase().as_str() {
            "trace" => Some(LogLevel::Trace),
            "debug" => Some(LogLevel::Debug),
            "info" => Some(LogLevel::Info),
            "warn" | "warning" => Some(LogLevel::Warn),
            "error" | "err" => Some(LogLevel::Error),
            _ => None,
        }
    }

    fn color(&self) -> Color {
        match self {
            LogLevel::Trace => Color::Magenta,
            LogLevel::Debug => Color::Cyan,
            LogLevel::Info => Color::Green,
            LogLevel::Warn => Color::Yellow,
            LogLevel::Error => Color::Red,
        }
    }

    /// Convert a `LogType` from the SHM ring buffer to a `LogLevel` for filtering.
    fn from_log_type(lt: &LogType) -> Self {
        match lt {
            LogType::Error => LogLevel::Error,
            LogType::Warning => LogLevel::Warn,
            LogType::Info => LogLevel::Info,
            LogType::Debug => LogLevel::Debug,
            // Publish/Subscribe IPC events sit below Info in severity
            LogType::Publish | LogType::Subscribe => LogLevel::Trace,
        }
    }
}

// ─── Time helpers ─────────────────────────────────────────────────────────────

/// Parse a "since" duration string (e.g. "5m", "1h", "30s") and return the
/// wall-clock threshold below which entries should be filtered out.
fn parse_since(since: Option<&str>) -> HorusResult<Option<SystemTime>> {
    let since = match since {
        Some(s) => s,
        None => return Ok(None),
    };

    let (num_str, unit) = if let Some(s) = since.strip_suffix('s') {
        (s, "s")
    } else if let Some(s) = since.strip_suffix('m') {
        (s, "m")
    } else if let Some(s) = since.strip_suffix('h') {
        (s, "h")
    } else if let Some(s) = since.strip_suffix('d') {
        (s, "d")
    } else {
        return Err(HorusError::Config(ConfigError::Other(format!(
            "Invalid time format: {}. Use '5m', '1h', '30s', '2d'",
            since
        ))));
    };

    let num: u64 = num_str.parse().map_err(|_| {
        HorusError::Config(ConfigError::Other(format!(
            "Invalid number in time: {}",
            since
        )))
    })?;

    let secs = match unit {
        "s" => num,
        "m" => num * 60,
        "h" => num * 3600,
        "d" => num * 86400,
        _ => {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Unknown time unit in: {}",
                since
            ))))
        }
    };

    Ok(Some(SystemTime::now() - secs.secs()))
}

/// Parse the `%H:%M:%S%.3f` timestamp stored in `LogEntry` into a `SystemTime`.
///
/// The ring buffer stores only the time-of-day component (no date). We combine
/// today's date with the parsed time. If parsing fails, `None` is returned and
/// the entry passes the since-filter by default.
fn parse_entry_time(ts: &str) -> Option<SystemTime> {
    use std::time::UNIX_EPOCH;
    let today = chrono::Local::now().date_naive();
    let fmt = "%H:%M:%S%.3f";
    let time = chrono::NaiveTime::parse_from_str(ts, fmt).ok()?;
    let dt = today.and_time(time);
    let epoch_secs = dt.and_utc().timestamp();
    if epoch_secs < 0 {
        return None;
    }
    Some(UNIX_EPOCH + (epoch_secs as u64).secs())
}

// ─── Display helpers ──────────────────────────────────────────────────────────

fn level_str(lt: &LogType) -> &'static str {
    match lt {
        LogType::Info => "INFO ",
        LogType::Warning => "WARN ",
        LogType::Error => "ERROR",
        LogType::Debug => "DEBUG",
        LogType::Publish => "PUB  ",
        LogType::Subscribe => "SUB  ",
    }
}

fn print_entry(entry: &LogEntry) {
    let level = LogLevel::from_log_type(&entry.log_type);
    let lstr = level_str(&entry.log_type).color(level.color());
    let node = format!("[{}]", entry.node_name).cyan();
    let ts = entry.timestamp.dimmed();

    if let Some(ref topic) = entry.topic {
        println!(
            "{} {} {} {} {}",
            ts,
            lstr,
            node,
            topic.cyan(),
            entry.message
        );
    } else {
        println!("{} {} {} {}", ts, lstr, node, entry.message);
    }
}

// ─── Main entry points ────────────────────────────────────────────────────────

/// View logs from the HORUS shared-memory ring buffer.
pub fn view_logs(
    node_filter: Option<&str>,
    level_filter: Option<&str>,
    since: Option<&str>,
    follow: bool,
    count: Option<usize>,
) -> HorusResult<()> {
    let min_level = level_filter
        .and_then(LogLevel::from_str)
        .unwrap_or(LogLevel::Trace);

    let since_time = parse_since(since)?;
    let max_entries = count.unwrap_or(100);

    println!("{}", "HORUS Log Viewer".green().bold());
    println!();

    if let Some(node) = node_filter {
        println!("  {} {}", "Filter node:".cyan(), node);
    }
    if let Some(level) = level_filter {
        println!(
            "  {} {} and above",
            "Filter level:".cyan(),
            level.to_uppercase()
        );
    }
    if let Some(s) = since {
        println!("  {} last {}", "Since:".cyan(), s);
    }
    println!();

    // Drain the ring buffer and apply filters
    let all = GLOBAL_LOG_BUFFER.get_all();

    if all.is_empty() {
        println!("{}", "No log entries found.".yellow());
        println!(
            "  {} Start a HORUS application to generate logs",
            "Tip:".dimmed()
        );
        if follow {
            println!();
            println!("{}", "Following (Ctrl+C to stop)...".dimmed());
            return follow_logs(node_filter, min_level);
        }
        return Ok(());
    }

    // Filter and collect the last `max_entries` matching entries
    let filtered: Vec<&LogEntry> = all
        .iter()
        .filter(|e| {
            // Level filter
            if LogLevel::from_log_type(&e.log_type) < min_level {
                return false;
            }
            // Node filter
            if let Some(node) = node_filter {
                if !e.node_name.contains(node) {
                    return false;
                }
            }
            // Since filter
            if let Some(since_wall) = since_time {
                if let Some(entry_wall) = parse_entry_time(&e.timestamp) {
                    if entry_wall < since_wall {
                        return false;
                    }
                }
            }
            true
        })
        .collect();

    let shown: Vec<&&LogEntry> = filtered
        .iter()
        .rev()
        .take(max_entries)
        .collect::<Vec<_>>()
        .into_iter()
        .rev()
        .collect();

    for entry in &shown {
        print_entry(entry);
    }

    if shown.is_empty() {
        println!("{}", "No log entries matched the filters.".dimmed());
    } else {
        println!();
        println!(
            "  {} {} entries shown",
            cli_output::ICON_HINT.dimmed(),
            shown.len()
        );
    }

    if follow {
        println!();
        println!("{}", "Following logs (Ctrl+C to stop)...".dimmed());
        follow_logs(node_filter, min_level)?;
    }

    Ok(())
}

/// Tail mode: poll the ring buffer, printing only new entries as they arrive.
///
/// Tracks the SHM `write_idx` counter so already-displayed entries are skipped.
fn follow_logs(node_filter: Option<&str>, min_level: LogLevel) -> HorusResult<()> {
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    // Snapshot the current write_idx so we only show *new* entries.
    let mut last_seen_idx = GLOBAL_LOG_BUFFER.write_idx();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        std::thread::sleep(100_u64.ms());

        let current_idx = GLOBAL_LOG_BUFFER.write_idx();
        if current_idx == last_seen_idx {
            continue;
        }

        // Fetch all and display only entries written after `last_seen_idx`.
        // write_idx is monotonically increasing (u64), so subtraction is safe.
        // Cap to the buffer's returned length to handle cases where more
        // entries arrived than the ring buffer can hold (older ones are lost).
        let all = GLOBAL_LOG_BUFFER.get_all();
        let new_count = (current_idx.wrapping_sub(last_seen_idx) as usize).min(all.len());
        let start = all.len().saturating_sub(new_count);

        for entry in &all[start..] {
            if LogLevel::from_log_type(&entry.log_type) < min_level {
                continue;
            }
            if let Some(node) = node_filter {
                if !entry.node_name.contains(node) {
                    continue;
                }
            }
            print_entry(entry);
        }

        last_seen_idx = current_idx;
    }

    println!();
    println!("{}", "Log following stopped.".dimmed());
    Ok(())
}

// ─── Clear logs ───────────────────────────────────────────────────────────────

/// Clear the HORUS shared-memory log ring buffer.
///
/// Deletes the SHM file at `shm_logs_path()`. The buffer will be recreated
/// automatically the next time any process writes a log entry. Running
/// processes that have already mmap'd the file continue to write to the old
/// inode (the mapping is not invalidated); those entries will not appear in
/// subsequent `horus log` invocations.
pub fn clear_logs(_all: bool) -> HorusResult<()> {
    let log_path = shm_logs_path();

    if log_path.exists() {
        println!(
            "{} Clearing log ring buffer at {}...",
            cli_output::ICON_INFO.cyan(),
            log_path.display()
        );
        std::fs::remove_file(&log_path).map_err(HorusError::Io)?;
        println!("{} Logs cleared.", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", "No log buffer found to clear.".dimmed());
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn log_level_from_str_valid() {
        assert_eq!(LogLevel::from_str("trace"), Some(LogLevel::Trace));
        assert_eq!(LogLevel::from_str("debug"), Some(LogLevel::Debug));
        assert_eq!(LogLevel::from_str("info"), Some(LogLevel::Info));
        assert_eq!(LogLevel::from_str("warn"), Some(LogLevel::Warn));
        assert_eq!(LogLevel::from_str("warning"), Some(LogLevel::Warn));
        assert_eq!(LogLevel::from_str("error"), Some(LogLevel::Error));
        assert_eq!(LogLevel::from_str("err"), Some(LogLevel::Error));
    }

    #[test]
    fn log_level_from_str_case_insensitive() {
        assert_eq!(LogLevel::from_str("INFO"), Some(LogLevel::Info));
        assert_eq!(LogLevel::from_str("Error"), Some(LogLevel::Error));
    }

    #[test]
    fn log_level_from_str_invalid() {
        assert_eq!(LogLevel::from_str("bogus"), None);
        assert_eq!(LogLevel::from_str(""), None);
    }

    #[test]
    fn log_level_ordering() {
        assert!(LogLevel::Trace < LogLevel::Debug);
        assert!(LogLevel::Debug < LogLevel::Info);
        assert!(LogLevel::Info < LogLevel::Warn);
        assert!(LogLevel::Warn < LogLevel::Error);
    }

    #[test]
    fn log_level_from_log_type() {
        assert_eq!(LogLevel::from_log_type(&LogType::Error), LogLevel::Error);
        assert_eq!(LogLevel::from_log_type(&LogType::Warning), LogLevel::Warn);
        assert_eq!(LogLevel::from_log_type(&LogType::Info), LogLevel::Info);
        assert_eq!(LogLevel::from_log_type(&LogType::Debug), LogLevel::Debug);
        assert_eq!(LogLevel::from_log_type(&LogType::Publish), LogLevel::Trace);
        assert_eq!(
            LogLevel::from_log_type(&LogType::Subscribe),
            LogLevel::Trace
        );
    }

    #[test]
    fn level_str_formatting() {
        assert_eq!(level_str(&LogType::Info), "INFO ");
        assert_eq!(level_str(&LogType::Warning), "WARN ");
        assert_eq!(level_str(&LogType::Error), "ERROR");
        assert_eq!(level_str(&LogType::Debug), "DEBUG");
        assert_eq!(level_str(&LogType::Publish), "PUB  ");
        assert_eq!(level_str(&LogType::Subscribe), "SUB  ");
    }

    #[test]
    fn parse_since_none_returns_none() {
        assert!(parse_since(None).unwrap().is_none());
    }

    #[test]
    fn parse_since_valid_formats() {
        assert!(parse_since(Some("5s")).unwrap().is_some());
        assert!(parse_since(Some("10m")).unwrap().is_some());
        assert!(parse_since(Some("2h")).unwrap().is_some());
        assert!(parse_since(Some("1d")).unwrap().is_some());
    }

    #[test]
    fn parse_since_invalid_format() {
        let err1 = parse_since(Some("5x")).unwrap_err().to_string();
        assert!(err1.contains("Invalid time format"), "should mention invalid format: {}", err1);
        assert!(err1.contains("5x"), "should include the invalid input: {}", err1);

        let err2 = parse_since(Some("abc")).unwrap_err().to_string();
        assert!(err2.contains("Invalid"), "should indicate invalidity: {}", err2);
    }

    #[test]
    fn parse_since_invalid_number() {
        let err = parse_since(Some("abcs")).unwrap_err().to_string();
        assert!(err.contains("Invalid number"), "should mention invalid number: {}", err);
        assert!(err.contains("abcs"), "should include the invalid input: {}", err);
    }

    #[test]
    fn parse_entry_time_valid() {
        let result = parse_entry_time("12:30:45.123");
        assert!(result.is_some());
    }

    #[test]
    fn parse_entry_time_invalid() {
        assert!(parse_entry_time("not-a-time").is_none());
        assert!(parse_entry_time("").is_none());
    }

    // ── Battle tests: LogLevel filtering ──────────────────────────────────

    #[test]
    fn log_level_from_str_mixed_case_variants() {
        assert_eq!(LogLevel::from_str("TRACE"), Some(LogLevel::Trace));
        assert_eq!(LogLevel::from_str("Debug"), Some(LogLevel::Debug));
        assert_eq!(LogLevel::from_str("wArN"), Some(LogLevel::Warn));
        assert_eq!(LogLevel::from_str("WARNING"), Some(LogLevel::Warn));
        assert_eq!(LogLevel::from_str("ERR"), Some(LogLevel::Error));
    }

    #[test]
    fn log_level_from_str_whitespace_returns_none() {
        assert_eq!(LogLevel::from_str(" info"), None);
        assert_eq!(LogLevel::from_str("info "), None);
        assert_eq!(LogLevel::from_str(" "), None);
        assert_eq!(LogLevel::from_str("\t"), None);
        assert_eq!(LogLevel::from_str("\n"), None);
    }

    #[test]
    fn log_level_from_str_partial_match_returns_none() {
        assert_eq!(LogLevel::from_str("inf"), None);
        assert_eq!(LogLevel::from_str("e"), None);
        assert_eq!(LogLevel::from_str("war"), None);
        assert_eq!(LogLevel::from_str("deb"), None);
        assert_eq!(LogLevel::from_str("trac"), None);
    }

    #[test]
    fn log_level_ordering_all_pairs() {
        let levels = [
            LogLevel::Trace,
            LogLevel::Debug,
            LogLevel::Info,
            LogLevel::Warn,
            LogLevel::Error,
        ];
        for i in 0..levels.len() {
            for j in (i + 1)..levels.len() {
                assert!(levels[i] < levels[j], "{:?} should be < {:?}", levels[i], levels[j]);
            }
        }
        // Equality
        for level in &levels {
            assert_eq!(*level, *level);
        }
    }

    #[test]
    fn log_level_color_returns_distinct_colors() {
        let levels = [
            LogLevel::Trace,
            LogLevel::Debug,
            LogLevel::Info,
            LogLevel::Warn,
            LogLevel::Error,
        ];
        // Each level should return a color (no panic)
        for level in &levels {
            let _ = level.color();
        }
        // Adjacent severity levels should have distinct colors
        assert_ne!(LogLevel::Info.color(), LogLevel::Warn.color());
        assert_ne!(LogLevel::Warn.color(), LogLevel::Error.color());
    }

    #[test]
    fn log_level_from_log_type_pub_sub_are_trace() {
        // Verify IPC events map to lowest severity
        assert_eq!(LogLevel::from_log_type(&LogType::Publish), LogLevel::Trace);
        assert_eq!(LogLevel::from_log_type(&LogType::Subscribe), LogLevel::Trace);
        // Both should be below Info
        assert!(LogLevel::from_log_type(&LogType::Publish) < LogLevel::Info);
    }

    // ── Battle tests: parse_since edge cases ──────────────────────────────

    #[test]
    fn parse_since_zero_value() {
        // "0s" = zero seconds ago = now
        let result = parse_since(Some("0s")).unwrap();
        assert!(result.is_some());
    }

    #[test]
    fn parse_since_large_values() {
        // Very large durations should not overflow
        assert!(parse_since(Some("999999s")).unwrap().is_some());
        assert!(parse_since(Some("1000m")).unwrap().is_some());
        assert!(parse_since(Some("365d")).unwrap().is_some());
    }

    #[test]
    fn parse_since_negative_number_fails() {
        let err = parse_since(Some("-5s")).unwrap_err().to_string();
        assert!(err.contains("Invalid number"), "negative should fail: {}", err);
    }

    #[test]
    fn parse_since_float_number_fails() {
        let err = parse_since(Some("1.5h")).unwrap_err().to_string();
        assert!(err.contains("Invalid number"), "float should fail: {}", err);
    }

    #[test]
    fn parse_since_empty_unit_fails() {
        let err = parse_since(Some("5")).unwrap_err().to_string();
        assert!(err.contains("Invalid time format"), "missing unit should fail: {}", err);
    }

    #[test]
    fn parse_since_only_unit_fails() {
        let err = parse_since(Some("s")).unwrap_err().to_string();
        assert!(err.contains("Invalid number"), "only-unit should fail: {}", err);
    }

    // ── Battle tests: parse_entry_time edge cases ─────────────────────────

    #[test]
    fn parse_entry_time_midnight() {
        let result = parse_entry_time("00:00:00.000");
        assert!(result.is_some());
    }

    #[test]
    fn parse_entry_time_end_of_day() {
        let result = parse_entry_time("23:59:59.999");
        assert!(result.is_some());
    }

    #[test]
    fn parse_entry_time_no_fractional() {
        // Format requires %.3f but chrono may or may not accept bare seconds
        let _result = parse_entry_time("12:30:45");
        // Just ensure no panic — behavior may vary
    }

    #[test]
    fn parse_entry_time_garbage_strings() {
        assert!(parse_entry_time("25:99:99.000").is_none());
        assert!(parse_entry_time("abc:de:fg.hij").is_none());
        assert!(parse_entry_time("12:30").is_none());
        assert!(parse_entry_time(":::").is_none());
        assert!(parse_entry_time("99999").is_none());
    }

    // ── Battle tests: level_str formatting ────────────────────────────────

    #[test]
    fn level_str_all_same_width_except_error() {
        // All should be 5 chars for alignment
        assert_eq!(level_str(&LogType::Info).len(), 5);
        assert_eq!(level_str(&LogType::Warning).len(), 5);
        assert_eq!(level_str(&LogType::Error).len(), 5);
        assert_eq!(level_str(&LogType::Debug).len(), 5);
        assert_eq!(level_str(&LogType::Publish).len(), 5);
        assert_eq!(level_str(&LogType::Subscribe).len(), 5);
    }

    // ── Battle tests: LogEntry construction for print_entry ───────────────

    #[test]
    fn print_entry_with_topic_does_not_panic() {
        let entry = LogEntry {
            timestamp: "12:00:00.000".to_string(),
            tick_number: 0,
            node_name: "test_node".to_string(),
            log_type: LogType::Info,
            topic: Some("/sensor/data".to_string()),
            message: "received 42 bytes".to_string(),
            tick_us: 100,
            ipc_ns: 200,
        };
        // Should not panic — output goes to stdout
        print_entry(&entry);
    }

    #[test]
    fn print_entry_without_topic_does_not_panic() {
        let entry = LogEntry {
            timestamp: "12:00:00.000".to_string(),
            tick_number: 0,
            node_name: "test_node".to_string(),
            log_type: LogType::Error,
            topic: None,
            message: "something failed".to_string(),
            tick_us: 100,
            ipc_ns: 0,
        };
        print_entry(&entry);
    }

    #[test]
    fn print_entry_special_characters_no_panic() {
        let entry = LogEntry {
            timestamp: "12:00:00.000".to_string(),
            tick_number: 0,
            node_name: "node/with/slashes".to_string(),
            log_type: LogType::Warning,
            topic: Some("topic with spaces & <special> chars".to_string()),
            message: "msg with \"quotes\" and \t tabs".to_string(),
            tick_us: 0,
            ipc_ns: 0,
        };
        print_entry(&entry);
    }

    #[test]
    fn print_entry_empty_fields_no_panic() {
        let entry = LogEntry {
            timestamp: String::new(),
            tick_number: 0,
            node_name: String::new(),
            log_type: LogType::Debug,
            topic: Some(String::new()),
            message: String::new(),
            tick_us: 0,
            ipc_ns: 0,
        };
        print_entry(&entry);
    }
}
