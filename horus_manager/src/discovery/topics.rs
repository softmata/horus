use super::*;

pub(super) fn discover_shared_memory_uncached() -> HorusResult<Vec<SharedMemoryInfo>> {
    use horus_core::communication::read_topic_header_info;
    use horus_core::memory::shm_topics_dir;

    // Delegate to horus_sys cross-platform discovery
    let sys_topics = horus_sys::discover::find_topics();
    let topics_dir = shm_topics_dir();

    let topics = sys_topics
        .into_iter()
        .map(|t| {
            let procs: Vec<u32> = if t.creator_pid > 0 && t.is_alive {
                vec![t.creator_pid]
            } else {
                vec![]
            };
            let (_status, age_string) = compute_topic_status(&procs, None);
            let status = if t.is_alive {
                TopicStatus::Active
            } else {
                TopicStatus::Stale
            };

            // Try to read header metadata directly from SHM file
            let topic_path = topics_dir.join(&t.name);
            let header_info = read_topic_header_info(&topic_path);

            // Use header type_name as primary, fall back to presence-based type
            let message_type = header_info
                .as_ref()
                .map(|h| &h.type_name)
                .filter(|n| !n.is_empty())
                .cloned()
                .or(t.message_type);

            let messages_total = header_info.as_ref().map(|h| h.messages_total).unwrap_or(0);
            let topic_kind = header_info.as_ref().map(|h| h.topic_kind).unwrap_or(0);

            SharedMemoryInfo {
                topic_name: t.name.clone(),
                size_bytes: t.size as u64,
                active: t.is_alive,
                accessing_processes: procs,
                last_modified: None,
                message_type,
                publishers: t.publishers,
                subscribers: t.subscribers,
                message_rate_hz: 0.0,
                status,
                age_string,
                is_system: t.name.starts_with("horus."),
                messages_total,
                topic_kind,
            }
        })
        .collect();

    Ok(topics)
}
/// Clean up stale topic files from the global topics directory.
///
/// A topic is considered stale if no process holds a flock on it.
/// The flock check is O(1) per file and race-free (survives SIGKILL).
///
/// Internal: Clean up stale topic files in a specific directory
pub(super) fn cleanup_stale_topics_in_dir(shm_path: &Path) {
    if let Ok(entries) = std::fs::read_dir(shm_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }

            if horus_core::memory::is_shm_file_stale(&path) {
                let _ = std::fs::remove_file(&path);
            }
        }
    }
}

/// Compute topic status and age string based on live processes and modification time
pub(super) fn compute_topic_status(
    live_processes: &[u32],
    modified: Option<std::time::SystemTime>,
) -> (TopicStatus, String) {
    const ACTIVE_THRESHOLD_SECS: u64 = 30; // Recent activity = within 30 seconds
    const STALE_THRESHOLD_SECS: u64 = 300; // Stale = 5+ minutes

    let has_live_processes = !live_processes.is_empty();

    // Calculate age
    let (age_secs, age_string) = if let Some(mod_time) = modified {
        if let Ok(elapsed) = mod_time.elapsed() {
            let secs = elapsed.as_secs();
            let age_str = format_age(secs);
            (Some(secs), age_str)
        } else {
            (None, "unknown".to_string())
        }
    } else {
        (None, "unknown".to_string())
    };

    // Determine status
    let status = match (has_live_processes, age_secs) {
        // Has live processes AND recent writes = Active
        (true, Some(secs)) if secs < ACTIVE_THRESHOLD_SECS => TopicStatus::Active,
        // Has live processes but no recent writes = Idle
        (true, _) => TopicStatus::Idle,
        // No live processes AND old = Stale
        (false, Some(secs)) if secs >= STALE_THRESHOLD_SECS => TopicStatus::Stale,
        // No live processes but recent = Idle (process just exited)
        (false, Some(_)) => TopicStatus::Idle,
        // Unknown modification time = Stale
        (false, None) => TopicStatus::Stale,
    };

    (status, age_string)
}

/// Format age in human-readable form
pub(super) fn format_age(secs: u64) -> String {
    if secs < 60 {
        format!("{}s ago", secs)
    } else if secs < 3600 {
        format!("{}m ago", secs / 60)
    } else if secs < 86400 {
        format!("{}h ago", secs / 3600)
    } else {
        format!("{}d ago", secs / 86400)
    }
}

/// Check whether a process command line indicates a HORUS-related process.
///
/// Uses targeted patterns to avoid false positives from unrelated programs
/// that happen to contain generic words like "sim" or "ros".
#[allow(dead_code)] // used by tests
pub(super) fn is_horus_process(cmdline: &str) -> bool {
    // Primary: horus binaries and libraries
    if cmdline.contains("horus") {
        return true;
    }
    // Horus sim3d
    if cmdline.contains("horus-sim3d") || cmdline.contains("horus_sim3d") {
        return true;
    }
    // Talos simulator
    if cmdline.contains("talos") {
        return true;
    }
    // Check for processes that have the SHM topics path open (definitive match)
    let shm_parent = horus_sys::shm::shm_parent_dir();
    if cmdline.contains(&shm_parent.to_string_lossy().as_ref()) {
        return true;
    }
    false
}

