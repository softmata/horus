//! Linux process discovery via /proc filesystem.

use super::ProcessInfo;
use std::collections::HashMap;
use std::sync::{OnceLock, RwLock};
use std::time::Instant;

/// CPU usage cache: PID → (total_jiffies, timestamp). Evicted when size exceeds 1024.
fn cpu_cache() -> &'static RwLock<HashMap<u32, (u64, Instant)>> {
    static CACHE: OnceLock<RwLock<HashMap<u32, (u64, Instant)>>> = OnceLock::new();
    CACHE.get_or_init(|| RwLock::new(HashMap::new()))
}

const CPU_CACHE_MAX_SIZE: usize = 1024;

/// Get detailed process info by reading /proc/{pid}/.
pub fn get_process_info(pid: u32) -> anyhow::Result<ProcessInfo> {
    let proc_path = format!("/proc/{}", pid);

    // Read command line
    let cmdline = std::fs::read_to_string(format!("{}/cmdline", proc_path))
        .unwrap_or_default()
        .replace('\0', " ")
        .trim()
        .to_string();

    // Read working directory
    let working_dir = std::fs::read_link(format!("{}/cwd", proc_path))
        .map(|p| p.to_string_lossy().to_string())
        .unwrap_or_else(|_| "/".to_string());

    // Read stat for memory and CPU info
    let stat_content = std::fs::read_to_string(format!("{}/stat", proc_path))?;
    let memory_kb = parse_memory_from_stat(&stat_content);
    let cpu_percent = calculate_cpu_usage(pid, &stat_content);
    let start_time = get_process_start_time(pid);

    Ok(ProcessInfo {
        pid,
        cmdline,
        working_dir,
        cpu_percent,
        memory_kb,
        start_time,
    })
}

/// List all process IDs by reading /proc/ directory entries.
pub fn list_pids() -> Vec<u32> {
    let mut pids = Vec::new();
    if let Ok(entries) = std::fs::read_dir("/proc") {
        for entry in entries.flatten() {
            if let Some(name) = entry.file_name().to_str() {
                if let Ok(pid) = name.parse::<u32>() {
                    pids.push(pid);
                }
            }
        }
    }
    pids
}

// ── /proc/[pid]/stat parsing ─────────────────────────────────────────────

/// Parse fields from `/proc/[pid]/stat` safely, handling process names
/// that contain spaces or parentheses.
///
/// The `comm` field (field 2) is enclosed in `()` and may contain arbitrary
/// characters including spaces and nested parentheses. We skip it by finding
/// the **last** `)` and then splitting the remainder by whitespace.
///
/// Returns the fields **after** the comm field (0-indexed: field 0 = state,
/// field 11 = utime, field 12 = stime, field 19 = starttime, field 21 = rss).
pub(crate) fn parse_stat_fields(stat_content: &str) -> Option<Vec<&str>> {
    let after_comm = stat_content.rfind(')')?.checked_add(1)?;
    if after_comm >= stat_content.len() {
        return None;
    }
    let fields: Vec<&str> = stat_content[after_comm..].split_whitespace().collect();
    if fields.is_empty() {
        return None;
    }
    Some(fields)
}

fn calculate_cpu_usage(pid: u32, stat_content: &str) -> f32 {
    // After comm: field[11] = utime, field[12] = stime
    let fields = match parse_stat_fields(stat_content) {
        Some(f) if f.len() > 12 => f,
        _ => return 0.0,
    };

    let utime = fields[11].parse::<u64>().unwrap_or(0);
    let stime = fields[12].parse::<u64>().unwrap_or(0);
    let total_time = utime + stime;

    if let Ok(mut cache) = cpu_cache().write() {
        let now = Instant::now();

        if let Some((prev_total, prev_time)) = cache.get(&pid) {
            let time_delta = now.duration_since(*prev_time).as_secs_f32();
            if time_delta > 0.0 {
                let cpu_delta = (total_time.saturating_sub(*prev_total)) as f32;
                // Convert from jiffies to percentage (100 Hz clock)
                let cpu_percent = (cpu_delta / time_delta / 100.0) * 100.0;
                cache.insert(pid, (total_time, now));
                return cpu_percent.min(100.0);
            }
        }

        // First sample: estimate lifetime average
        // Evict stale entries (older than 60s) if cache is too large
        if cache.len() >= CPU_CACHE_MAX_SIZE {
            let cutoff = Instant::now() - std::time::Duration::from_secs(60);
            cache.retain(|_, (_, ts)| *ts > cutoff);
        }
        cache.insert(pid, (total_time, now));

        // fields[19] = starttime (jiffies since boot)
        if fields.len() > 19 {
            if let Ok(start_jiffies) = fields[19].parse::<u64>() {
                if let Ok(uptime_str) = std::fs::read_to_string("/proc/uptime") {
                    if let Some(uptime_secs) = uptime_str
                        .split_whitespace()
                        .next()
                        .and_then(|s| s.parse::<f64>().ok())
                    {
                        let uptime_jiffies = (uptime_secs * 100.0) as u64;
                        let process_jiffies = uptime_jiffies.saturating_sub(start_jiffies);
                        if process_jiffies > 0 {
                            let pct = (total_time as f32 / process_jiffies as f32) * 100.0;
                            return pct.min(100.0);
                        }
                    }
                }
            }
        }
    }

    0.0
}

pub(crate) fn parse_memory_from_stat(stat: &str) -> u64 {
    // After comm: field[21] = rss (in pages)
    let fields = match parse_stat_fields(stat) {
        Some(f) if f.len() > 21 => f,
        _ => return 0,
    };

    if let Ok(rss_pages) = fields[21].parse::<u64>() {
        let page_size_kb = page_size_kb();
        return rss_pages * page_size_kb;
    }
    0
}

/// System page size in KB, cached (never changes at runtime).
fn page_size_kb() -> u64 {
    use std::sync::atomic::{AtomicU64, Ordering};
    static CACHED: AtomicU64 = AtomicU64::new(0);
    let cached = CACHED.load(Ordering::Relaxed);
    if cached != 0 {
        return cached;
    }
    // SAFETY: sysconf(_SC_PAGESIZE) is always safe and returns the page size in bytes.
    let bytes = unsafe { libc::sysconf(libc::_SC_PAGESIZE) };
    let kb = if bytes > 0 { bytes as u64 / 1024 } else { 4 };
    CACHED.store(kb, Ordering::Relaxed);
    kb
}

fn get_process_start_time(pid: u32) -> String {
    if let Ok(stat) = std::fs::read_to_string(format!("/proc/{}/stat", pid)) {
        if let Some(fields) = parse_stat_fields(&stat) {
            if fields.len() > 19 {
                if let Ok(start_jiffies) = fields[19].parse::<u64>() {
                    let start_secs = start_jiffies / 100;
                    return super::format_duration(std::time::Duration::from_secs(start_secs));
                }
            }
        }
    }
    "Unknown".to_string()
}
