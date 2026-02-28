//! Log command - View and filter HORUS logs
//!
//! Provides real-time log viewing from shared memory and log files.

use crate::cli_output;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use horus_core::memory::shm_base_dir;
use std::collections::VecDeque;
use std::fs::{self, File};
use std::io::{BufRead, BufReader, Seek, SeekFrom};
use std::path::{Path, PathBuf};
use std::time::{Duration, SystemTime, UNIX_EPOCH};

/// Log level for filtering
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
}

/// Log entry
#[derive(Debug, Clone)]
struct LogEntry {
    timestamp: SystemTime,
    level: LogLevel,
    node: String,
    message: String,
    _raw: String,
}

/// View logs (tail mode)
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

    // Try to read from shared memory log buffer first
    let shm_base = shm_base_dir();
    let log_path = Path::new(&shm_base).join("logs");

    if log_path.exists() {
        // Read from shared memory log files
        view_shm_logs(&log_path, node_filter, min_level, since_time, follow, count)?;
    } else {
        // Fallback to standard log files
        let log_dirs = find_log_directories()?;
        if log_dirs.is_empty() {
            println!("{}", "No log files found.".yellow());
            println!(
                "  {} Start a HORUS application to generate logs",
                "Tip:".dimmed()
            );
            return Ok(());
        }
        view_file_logs(&log_dirs, node_filter, min_level, since_time, follow, count)?;
    }

    Ok(())
}

/// Parse "since" duration string (e.g., "5m", "1h", "30s")
fn parse_since(since: Option<&str>) -> HorusResult<Option<SystemTime>> {
    let since = match since {
        Some(s) => s,
        None => return Ok(None),
    };

    let (num_str, unit) = if let Some(stripped) = since.strip_suffix('s') {
        (stripped, "s")
    } else if let Some(stripped) = since.strip_suffix('m') {
        (stripped, "m")
    } else if let Some(stripped) = since.strip_suffix('h') {
        (stripped, "h")
    } else if let Some(stripped) = since.strip_suffix('d') {
        (stripped, "d")
    } else {
        return Err(HorusError::Config(format!(
            "Invalid time format: {}. Use format like '5m', '1h', '30s'",
            since
        )));
    };

    let num: u64 = num_str
        .parse()
        .map_err(|_| HorusError::Config(format!("Invalid number in time: {}", since)))?;

    let secs = match unit {
        "s" => num,
        "m" => num * 60,
        "h" => num * 3600,
        "d" => num * 86400,
        _ => unreachable!(),
    };

    Ok(Some(SystemTime::now() - Duration::from_secs(secs)))
}

/// View logs from shared memory
fn view_shm_logs(
    log_path: &Path,
    node_filter: Option<&str>,
    min_level: LogLevel,
    since: Option<SystemTime>,
    follow: bool,
    count: Option<usize>,
) -> HorusResult<()> {
    let mut entries: VecDeque<LogEntry> = VecDeque::new();
    let max_entries = count.unwrap_or(100);

    // Read all log files in the directory
    if let Ok(read_dir) = fs::read_dir(log_path) {
        for entry in read_dir.flatten() {
            let path = entry.path();
            if path.extension().map(|e| e == "log").unwrap_or(false) {
                if let Ok(file) = File::open(&path) {
                    let reader = BufReader::new(file);
                    for line in reader.lines().map_while(Result::ok) {
                        if let Some(entry) = parse_log_line(&line) {
                            // Apply filters
                            if entry.level < min_level {
                                continue;
                            }
                            if let Some(node) = node_filter {
                                if !entry.node.contains(node) {
                                    continue;
                                }
                            }
                            if let Some(since_time) = since {
                                if entry.timestamp < since_time {
                                    continue;
                                }
                            }
                            entries.push_back(entry);
                            if entries.len() > max_entries {
                                entries.pop_front();
                            }
                        }
                    }
                }
            }
        }
    }

    // Print entries
    for entry in &entries {
        print_log_entry(entry);
    }

    if entries.is_empty() {
        println!("{}", "No log entries found matching filters.".dimmed());
    } else {
        println!();
        println!("  {} {} entries shown", cli_output::ICON_HINT.dimmed(), entries.len());
    }

    // Follow mode
    if follow {
        println!();
        println!("{}", "Following logs (Ctrl+C to stop)...".dimmed());
        follow_logs(log_path, node_filter, min_level)?;
    }

    Ok(())
}

/// View logs from standard log files
fn view_file_logs(
    log_dirs: &[PathBuf],
    node_filter: Option<&str>,
    min_level: LogLevel,
    since: Option<SystemTime>,
    follow: bool,
    count: Option<usize>,
) -> HorusResult<()> {
    let mut entries: VecDeque<LogEntry> = VecDeque::new();
    let max_entries = count.unwrap_or(100);

    for dir in log_dirs {
        if let Ok(read_dir) = fs::read_dir(dir) {
            for entry in read_dir.flatten() {
                let path = entry.path();
                if path.extension().map(|e| e == "log").unwrap_or(false) {
                    if let Ok(file) = File::open(&path) {
                        let reader = BufReader::new(file);
                        for line in reader.lines().map_while(Result::ok) {
                            if let Some(entry) = parse_log_line(&line) {
                                if entry.level < min_level {
                                    continue;
                                }
                                if let Some(node) = node_filter {
                                    if !entry.node.contains(node) {
                                        continue;
                                    }
                                }
                                if let Some(since_time) = since {
                                    if entry.timestamp < since_time {
                                        continue;
                                    }
                                }
                                entries.push_back(entry);
                                if entries.len() > max_entries {
                                    entries.pop_front();
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    for entry in &entries {
        print_log_entry(entry);
    }

    if entries.is_empty() {
        println!("{}", "No log entries found matching filters.".dimmed());
    } else {
        println!();
        println!("  {} {} entries shown", cli_output::ICON_HINT.dimmed(), entries.len());
    }

    if follow && !log_dirs.is_empty() {
        println!();
        println!("{}", "Following logs (Ctrl+C to stop)...".dimmed());
        follow_logs(&log_dirs[0], node_filter, min_level)?;
    }

    Ok(())
}

/// Parse a log line into a LogEntry
fn parse_log_line(line: &str) -> Option<LogEntry> {
    // Try to parse common log formats
    // Format 1: [2024-01-01T12:00:00] [INFO] [node_name] message
    // Format 2: 2024-01-01 12:00:00 INFO node_name: message
    // Format 3: [INFO] message
    // Format 4: INFO message

    let line = line.trim();
    if line.is_empty() {
        return None;
    }

    // Try format: [timestamp] [LEVEL] [node] message
    if line.starts_with('[') {
        let parts: Vec<&str> = line.splitn(4, ']').collect();
        if parts.len() >= 3 {
            let timestamp_str = parts[0].trim_start_matches('[').trim();
            let level_str = parts[1].trim_start_matches(" [").trim();
            let rest = if parts.len() > 3 {
                parts[2..].join("]")
            } else {
                parts[2].to_string()
            };

            let level = LogLevel::from_str(level_str)?;

            // Try to extract node name
            let (node, message) = if rest.starts_with(" [") {
                if let Some(end) = rest.find(']') {
                    let node = rest[2..end].to_string();
                    let msg = rest[end + 1..].trim().to_string();
                    (node, msg)
                } else {
                    ("unknown".to_string(), rest.trim().to_string())
                }
            } else {
                ("unknown".to_string(), rest.trim().to_string())
            };

            // Parse timestamp
            let timestamp = parse_timestamp(timestamp_str).unwrap_or_else(SystemTime::now);

            return Some(LogEntry {
                timestamp,
                level,
                node,
                message,
                _raw: line.to_string(),
            });
        }
    }

    // Try format: LEVEL node: message or LEVEL message
    let parts: Vec<&str> = line.splitn(2, char::is_whitespace).collect();
    if parts.len() >= 2 {
        if let Some(level) = LogLevel::from_str(parts[0]) {
            let rest = parts[1].trim();
            let (node, message) = if let Some(colon_pos) = rest.find(':') {
                let potential_node = &rest[..colon_pos];
                // Only treat as node if it looks like an identifier
                if potential_node
                    .chars()
                    .all(|c| c.is_alphanumeric() || c == '_' || c == '-')
                {
                    (
                        potential_node.to_string(),
                        rest[colon_pos + 1..].trim().to_string(),
                    )
                } else {
                    ("unknown".to_string(), rest.to_string())
                }
            } else {
                ("unknown".to_string(), rest.to_string())
            };

            return Some(LogEntry {
                timestamp: SystemTime::now(),
                level,
                node,
                message,
                _raw: line.to_string(),
            });
        }
    }

    // Fallback: treat as info message
    Some(LogEntry {
        timestamp: SystemTime::now(),
        level: LogLevel::Info,
        node: "unknown".to_string(),
        message: line.to_string(),
        _raw: line.to_string(),
    })
}

/// Parse timestamp string
fn parse_timestamp(s: &str) -> Option<SystemTime> {
    // Try ISO 8601 format
    if let Ok(dt) = chrono::DateTime::parse_from_rfc3339(s) {
        return Some(UNIX_EPOCH + Duration::from_secs(dt.timestamp() as u64));
    }
    // Try common formats
    if let Ok(dt) = chrono::NaiveDateTime::parse_from_str(s, "%Y-%m-%d %H:%M:%S") {
        return Some(UNIX_EPOCH + Duration::from_secs(dt.and_utc().timestamp() as u64));
    }
    if let Ok(dt) = chrono::NaiveDateTime::parse_from_str(s, "%Y-%m-%dT%H:%M:%S") {
        return Some(UNIX_EPOCH + Duration::from_secs(dt.and_utc().timestamp() as u64));
    }
    None
}

/// Print a log entry with coloring
fn print_log_entry(entry: &LogEntry) {
    let level_str = match entry.level {
        LogLevel::Trace => "TRACE",
        LogLevel::Debug => "DEBUG",
        LogLevel::Info => "INFO ",
        LogLevel::Warn => "WARN ",
        LogLevel::Error => "ERROR",
    };

    let timestamp = format_timestamp(entry.timestamp);
    let level_colored = level_str.color(entry.level.color());

    println!(
        "{} {} {} {}",
        timestamp.dimmed(),
        level_colored,
        format!("[{}]", entry.node).cyan(),
        entry.message
    );
}

/// Format timestamp for display
fn format_timestamp(time: SystemTime) -> String {
    let duration = time.duration_since(UNIX_EPOCH).unwrap_or_default();
    let secs = duration.as_secs();
    let datetime =
        chrono::DateTime::from_timestamp(secs as i64, 0).unwrap_or_else(chrono::Utc::now);
    datetime.format("%H:%M:%S").to_string()
}

/// Follow logs in real-time
fn follow_logs(log_path: &Path, node_filter: Option<&str>, min_level: LogLevel) -> HorusResult<()> {
    // Find the most recent log file
    let mut latest_file: Option<PathBuf> = None;
    let mut latest_time: Option<SystemTime> = None;

    if let Ok(read_dir) = fs::read_dir(log_path) {
        for entry in read_dir.flatten() {
            let path = entry.path();
            if path.extension().map(|e| e == "log").unwrap_or(false) {
                if let Ok(meta) = fs::metadata(&path) {
                    if let Ok(modified) = meta.modified() {
                        if latest_time.map(|t| modified > t).unwrap_or(true) {
                            latest_time = Some(modified);
                            latest_file = Some(path);
                        }
                    }
                }
            }
        }
    }

    let log_file = latest_file
        .ok_or_else(|| HorusError::Config("No log files found to follow".to_string()))?;

    // Open file and seek to end
    let mut file = File::open(&log_file).map_err(HorusError::Io)?;
    file.seek(SeekFrom::End(0)).map_err(HorusError::Io)?;

    let mut reader = BufReader::new(file);
    let mut line = String::new();

    // Set up Ctrl+C handler
    let running = std::sync::Arc::new(std::sync::atomic::AtomicBool::new(true));
    let r = running.clone();
    ctrlc::set_handler(move || {
        r.store(false, std::sync::atomic::Ordering::SeqCst);
    })
    .ok();

    while running.load(std::sync::atomic::Ordering::SeqCst) {
        line.clear();
        match reader.read_line(&mut line) {
            Ok(0) => {
                // No new data, wait a bit
                std::thread::sleep(Duration::from_millis(100));
            }
            Ok(_) => {
                if let Some(entry) = parse_log_line(&line) {
                    if entry.level >= min_level
                        && node_filter.map(|n| entry.node.contains(n)).unwrap_or(true)
                    {
                        print_log_entry(&entry);
                    }
                }
            }
            Err(_) => break,
        }
    }

    println!();
    println!("{}", "Log following stopped.".dimmed());
    Ok(())
}

/// Find directories that might contain log files
fn find_log_directories() -> HorusResult<Vec<PathBuf>> {
    let mut dirs = Vec::new();

    // Check common log locations
    let locations = [
        "logs",
        "log",
        ".horus/logs",
        "/var/log/horus",
        "/tmp/horus/logs",
    ];

    for loc in locations {
        let path = PathBuf::from(loc);
        if path.exists() && path.is_dir() {
            dirs.push(path);
        }
    }

    // Check home directory
    if let Some(home) = dirs::home_dir() {
        let horus_logs = home.join(".horus").join("logs");
        if horus_logs.exists() {
            dirs.push(horus_logs);
        }
    }

    Ok(dirs)
}

/// Clear logs
pub fn clear_logs(all: bool) -> HorusResult<()> {
    let shm_base = shm_base_dir();
    let log_path = Path::new(&shm_base).join("logs");

    let mut cleared = false;

    if log_path.exists() {
        println!(
            "{} Clearing shared memory logs at {}...",
            cli_output::ICON_INFO.cyan(),
            log_path.display()
        );
        fs::remove_dir_all(&log_path).map_err(HorusError::Io)?;
        fs::create_dir_all(&log_path).map_err(HorusError::Io)?;
        cleared = true;
    }

    if all {
        // Also clear file-based logs
        let log_dirs = find_log_directories()?;
        for dir in log_dirs {
            if dir.exists() {
                println!("{} Clearing logs at {}...", cli_output::ICON_INFO.cyan(), dir.display());
                if let Ok(entries) = fs::read_dir(&dir) {
                    for entry in entries.flatten() {
                        let path = entry.path();
                        if path.extension().map(|e| e == "log").unwrap_or(false) {
                            let _ = fs::remove_file(&path);
                        }
                    }
                }
                cleared = true;
            }
        }
    }

    if cleared {
        println!("{} Logs cleared.", cli_output::ICON_SUCCESS.green());
    } else {
        println!("{}", "No logs found to clear.".dimmed());
    }

    Ok(())
}
