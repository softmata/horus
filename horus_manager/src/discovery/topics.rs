use super::nodes::{discover_registry_files, process_exists};
use super::*;

use std::collections::HashMap as StdHashMap;
use horus_core::core::DurationExt;
use horus_core::communication::read_topic_sequence;

lazy_static::lazy_static! {
    /// Cache for sequence-based rate calculation: topic_name → (instant, sequence_counter)
    static ref TOPIC_RATE_CACHE: Arc<RwLock<StdHashMap<String, (Instant, u64)>>> =
        Arc::new(RwLock::new(StdHashMap::new()));
}

pub(super) fn discover_shared_memory_uncached() -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();
    let mut seen_topics: std::collections::HashSet<String> = std::collections::HashSet::new();

    // PRIMARY: Scan SHM topics directory (has full metadata: size, mtime, accessing processes)
    let topics_path = shm_topics_dir();
    if topics_path.exists() {
        match scan_topics_directory(&topics_path) {
            Ok(scanned) => {
                for topic in scanned {
                    seen_topics.insert(topic.topic_name.clone());
                    topics.push(topic);
                }
            }
            Err(e) => {
                eprintln!("[horus] WARNING: topic scan failed for {}: {}", topics_path.display(), e);
            }
        }
    }

    // SUPPLEMENT: Merge presence file data into existing topics or add new ones.
    // Presence files have accurate pub/sub relationships; SHM has size/rate metadata.
    // For duplicates, merge pub/sub info. For new topics, add them.
    for presence_topic in discover_topics_from_presence() {
        // Find matching existing topic (exact name or with/without horus_topic/ prefix)
        let matching_idx = topics.iter().position(|t| {
            t.topic_name == presence_topic.topic_name
                || t.topic_name == format!("horus_topic/{}", presence_topic.topic_name)
                || presence_topic
                    .topic_name
                    .strip_prefix("horus_topic/")
                    .map(|base| t.topic_name == base)
                    .unwrap_or(false)
        });

        if let Some(idx) = matching_idx {
            // Merge pub/sub info from presence into the SHM entry
            let existing = &mut topics[idx];
            for pub_name in &presence_topic.publishers {
                if !existing.publishers.contains(pub_name) {
                    existing.publishers.push(pub_name.clone());
                }
            }
            for sub_name in &presence_topic.subscribers {
                if !existing.subscribers.contains(sub_name) {
                    existing.subscribers.push(sub_name.clone());
                }
            }
            // Fill in message type if SHM entry doesn't have one
            if existing.message_type.is_none() {
                existing.message_type = presence_topic.message_type.clone();
            }
        } else {
            // New topic (e.g. network-only) — add it
            seen_topics.insert(presence_topic.topic_name.clone());
            topics.push(presence_topic);
        }
    }

    Ok(topics)
}

/// Discover topics from node presence files
/// This captures ALL topics including network-based ones that don't have SHM files
fn discover_topics_from_presence() -> Vec<SharedMemoryInfo> {
    use std::collections::HashMap;

    // Aggregate topic info from all presence files
    type TopicInfo = (Vec<String>, Vec<String>, Option<String>);
    let mut topic_map: HashMap<String, TopicInfo> = HashMap::new();

    for presence in super::cached_presence() {
        // Add publishers
        for pub_topic in presence.publishers() {
            let entry = topic_map
                .entry(pub_topic.topic_name.clone())
                .or_insert_with(|| (Vec::new(), Vec::new(), None));
            if !entry.0.contains(&presence.name().to_string()) {
                entry.0.push(presence.name().to_string());
            }
            // Capture type name if available
            if entry.2.is_none()
                && !pub_topic.type_name.is_empty()
                && pub_topic.type_name != "unknown"
            {
                entry.2 = Some(pub_topic.type_name.clone());
            }
        }

        // Add subscribers
        for sub_topic in presence.subscribers() {
            let entry = topic_map
                .entry(sub_topic.topic_name.clone())
                .or_insert_with(|| (Vec::new(), Vec::new(), None));
            if !entry.1.contains(&presence.name().to_string()) {
                entry.1.push(presence.name().to_string());
            }
            // Capture type name if available
            if entry.2.is_none()
                && !sub_topic.type_name.is_empty()
                && sub_topic.type_name != "unknown"
            {
                entry.2 = Some(sub_topic.type_name.clone());
            }
        }
    }

    // Convert to SharedMemoryInfo (network topics won't have SHM-specific metadata)
    topic_map
        .into_iter()
        .map(|(topic_name, (publishers, subscribers, message_type))| {
            let has_activity = !publishers.is_empty() || !subscribers.is_empty();
            let is_system = topic_name.starts_with("horus.");
            SharedMemoryInfo {
                topic_name,
                size_bytes: 0, // Network topics don't have SHM size
                active: has_activity,
                accessing_processes: Vec::new(), // Can't determine for network topics
                last_modified: None,
                message_type,
                publishers,
                subscribers,
                message_rate_hz: 0.0, // Can't measure without SHM
                status: if has_activity {
                    TopicStatus::Active
                } else {
                    TopicStatus::Stale
                },
                age_string: "network".to_string(), // Indicate this is from presence, not SHM
                is_system,
            }
        })
        .collect()
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

/// Scan a specific topics directory for shared memory files
fn scan_topics_directory(shm_path: &Path) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    // Load registry to get topic metadata (pub/sub relationships from scheduler)
    let registry_topics = load_topic_metadata_from_registry();

    let entries = match std::fs::read_dir(shm_path) {
        Ok(e) => e,
        Err(e) => {
            // Permission denied or other I/O error — return empty rather than
            // aborting all discovery.  The directory exists but we can't read it.
            eprintln!(
                "[horus] WARNING: cannot read topics directory {}: {}",
                shm_path.display(),
                e
            );
            return Ok(Vec::new());
        }
    };

    for entry in entries {
        let entry = match entry {
            Ok(e) => e,
            Err(_) => continue,
        };
        let path = entry.path();
        let metadata = match entry.metadata() {
            Ok(m) => m,
            Err(_) => continue,
        };

        // Smart filter for shared memory segments
        if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
            if metadata.is_file() {
                // Hub topics - files directly in topics directory
                if let Some(info) = scan_topic_file(&path, name, &registry_topics) {
                    topics.push(info);
                }
            } else if metadata.is_dir() && name == "horus_links" {
                // Link topics - files inside horus_links subdirectory
                if let Ok(link_topics) = scan_links_directory(&path, &registry_topics) {
                    topics.extend(link_topics);
                }
            } else if metadata.is_dir() && name == "horus_topic" {
                // Topic API topics - files inside horus_topic subdirectory
                // These are created by Topic::new() in HORUS applications
                if let Ok(api_topics) = scan_topic_api_directory(&path, &registry_topics) {
                    topics.extend(api_topics);
                }
            }
        }
    }

    Ok(topics)
}

/// Scan the horus_links directory for Link shared memory files
fn scan_links_directory(
    links_path: &Path,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    if let Ok(entries) = std::fs::read_dir(links_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
                        // Link topic name format: links/<topic>
                        let topic_name = format!("links/{}", name);
                        if let Some(mut info) =
                            scan_topic_file(&path, name, registry_topics)
                        {
                            info.topic_name = topic_name;
                            topics.push(info);
                        }
                    }
                }
            }
        }
    }

    Ok(topics)
}

/// Scan the horus_topic directory for Topic API shared memory files
/// These are created by Topic::new() in HORUS applications
fn scan_topic_api_directory(
    topic_path: &Path,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    if let Ok(entries) = std::fs::read_dir(topic_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
                        // Topic API uses horus_topic/<name> path format
                        // Include the subdirectory in topic_name for proper path construction
                        let topic_name = format!("horus_topic/{}", name);
                        if let Some(mut info) =
                            scan_topic_file(&path, name, registry_topics)
                        {
                            info.topic_name = topic_name;
                            topics.push(info);
                        }
                    }
                }
            }
        }
    }

    Ok(topics)
}

/// Scan a single topic file and create SharedMemoryInfo
fn scan_topic_file(
    path: &Path,
    name: &str,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
) -> Option<SharedMemoryInfo> {
    let metadata = std::fs::metadata(path).ok()?;
    let size = metadata.len();
    let modified = metadata.modified().ok();

    // Find processes accessing this segment (optimized)
    let accessing_procs = find_accessing_processes_fast(path, name);

    // All files in HORUS directory are valid topics
    // Extract topic name from filename (remove "horus_" prefix)
    // Topic names use dot notation (e.g., "motors.cmd_vel") - no conversion needed
    let topic_name = if name.starts_with("horus_") {
        name.strip_prefix("horus_").unwrap_or(name).to_string()
    } else {
        name.to_string()
    };

    let is_recent = if let Some(mod_time) = modified {
        // Use 30 second threshold to handle slow publishers (e.g., 0.1 Hz = 10 sec between publishes)
        mod_time.elapsed().unwrap_or(3600_u64.secs()) < 30_u64.secs()
    } else {
        false
    };

    let has_valid_processes = accessing_procs.iter().any(|pid| process_exists(*pid));

    // Include all topics in HORUS directory
    // Topics persist until manually cleaned or processes terminate
    let active = has_valid_processes || is_recent;

    // Calculate message rate from SHM sequence counter (accurate)
    let message_rate = calculate_topic_rate(&topic_name, path);

    // Get metadata from registry (pub/sub info from scheduler's registry.json)
    // Additional pub/sub info from presence files is merged in discover_shared_memory_uncached()
    let (message_type, publishers, subscribers) = registry_topics
        .get(&topic_name)
        .map(|(t, p, s)| (Some(t.clone()), p.clone(), s.clone()))
        .unwrap_or((None, Vec::new(), Vec::new()));

    // Compute live processes (filter dead ones)
    let live_processes: Vec<u32> = accessing_procs
        .iter()
        .filter(|pid| process_exists(**pid))
        .copied()
        .collect();

    // Compute status and age string
    let (status, age_string) = compute_topic_status(&live_processes, modified);

    let is_system = topic_name.starts_with("horus.");

    Some(SharedMemoryInfo {
        topic_name,
        size_bytes: size,
        active,
        accessing_processes: live_processes,
        last_modified: modified,
        message_type,
        publishers,
        subscribers,
        message_rate_hz: message_rate,
        status,
        age_string,
        is_system,
    })
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

/// Calculate message rate using the SHM header's sequence counter.
///
/// Reads the write-sequence from the topic header and computes the rate
/// as `(current_seq - prev_seq) / time_delta`.  Falls back to 0.0 on the
/// first sample or when the header is unreadable.
fn calculate_topic_rate(topic_name: &str, shm_path: &Path) -> f32 {
    let seq = match read_topic_sequence(shm_path) {
        Some(s) => s,
        None => return 0.0,
    };

    let now = Instant::now();

    if let Ok(mut cache) = TOPIC_RATE_CACHE.write() {
        if let Some((prev_instant, prev_seq)) = cache.get(topic_name) {
            let time_delta = now.duration_since(*prev_instant).as_secs_f32();
            if time_delta > 0.05 && seq > *prev_seq {
                let msg_delta = (seq - *prev_seq) as f32;
                let rate = msg_delta / time_delta;
                cache.insert(topic_name.to_string(), (now, seq));
                return rate;
            }
            // Too soon or no new messages — return previous rate estimate
            if time_delta < 0.05 {
                // Avoid dividing by near-zero; keep cache, return 0
                return 0.0;
            }
        }

        // First sample — seed the cache
        cache.insert(topic_name.to_string(), (now, seq));
    }

    0.0
}

fn load_topic_metadata_from_registry() -> StdHashMap<String, (String, Vec<String>, Vec<String>)> {
    let mut topic_map = StdHashMap::new();

    // Load from all registry files (supports multiple schedulers)
    let registry_files = discover_registry_files();

    for registry_path in registry_files {
        if let Ok(content) = std::fs::read_to_string(&registry_path) {
            if let Ok(registry) = serde_json::from_str::<serde_json::Value>(&content) {
                // Skip if scheduler is dead
                let scheduler_pid = registry["pid"].as_u64().unwrap_or(0) as u32;
                if !process_exists(scheduler_pid) {
                    continue;
                }

                if let Some(nodes) = registry["nodes"].as_array() {
                    for node in nodes {
                        let node_name = node["name"].as_str().unwrap_or("Unknown");

                        // Process publishers
                        if let Some(pubs) = node["publishers"].as_array() {
                            for pub_info in pubs {
                                if let (Some(topic), Some(type_name)) =
                                    (pub_info["topic"].as_str(), pub_info["type"].as_str())
                                {
                                    let entry = topic_map.entry(topic.to_string()).or_insert((
                                        type_name.to_string(),
                                        Vec::new(),
                                        Vec::new(),
                                    ));
                                    entry.1.push(node_name.to_string());
                                }
                            }
                        }

                        // Process subscribers
                        if let Some(subs) = node["subscribers"].as_array() {
                            for sub_info in subs {
                                if let (Some(topic), Some(type_name)) =
                                    (sub_info["topic"].as_str(), sub_info["type"].as_str())
                                {
                                    let entry = topic_map.entry(topic.to_string()).or_insert((
                                        type_name.to_string(),
                                        Vec::new(),
                                        Vec::new(),
                                    ));
                                    entry.2.push(node_name.to_string());
                                }
                            }
                        }
                    }
                }
            }
        }
    }

    topic_map
}

/// Check whether a process command line indicates a HORUS-related process.
///
/// Uses targeted patterns to avoid false positives from unrelated programs
/// that happen to contain generic words like "sim" or "ros".
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
    if cmdline.contains("/dev/shm/horus") {
        return true;
    }
    false
}

// Fast version: Check memory maps for HORUS processes to find mmap'd shared memory
fn find_accessing_processes_fast(shm_path: &Path, shm_name: &str) -> Vec<u32> {
    let mut processes = Vec::new();
    let shm_path_str = shm_path.to_string_lossy();

    // For HORUS-like shared memory, only check HORUS processes first (much faster)
    // Check both the name and the full path - topic files like "motors.cmd_vel" may not
    // contain "horus" in the name but the path /dev/shm/horus/topics/... does
    let is_horus_shm = shm_name.contains("horus")
        || shm_name.contains("topic")
        || shm_name.starts_with("ros")
        || shm_name.starts_with("shm_")
        || shm_path_str.contains("horus")
        || shm_path_str.contains("/topics/");

    if is_horus_shm {
        // Fast path: Only check processes with HORUS in their name
        if let Ok(proc_entries) = std::fs::read_dir("/proc") {
            for entry in proc_entries.flatten() {
                if let Some(pid_str) = entry.file_name().to_str() {
                    if let Ok(pid) = pid_str.parse::<u32>() {
                        // Quick check if this is a HORUS-related process
                        if let Ok(cmdline) = std::fs::read_to_string(entry.path().join("cmdline")) {
                            let cmdline_str = cmdline.replace('\0', " ");
                            if is_horus_process(&cmdline_str) {
                                // Check memory maps for this process (mmap'd files show up here)
                                let maps_path = entry.path().join("maps");
                                if let Ok(maps_content) = std::fs::read_to_string(&maps_path) {
                                    if maps_content.contains(&*shm_path_str) {
                                        processes.push(pid);
                                        continue;
                                    }
                                }
                                // Also check file descriptors as fallback
                                let fd_path = entry.path().join("fd");
                                if let Ok(fd_entries) = std::fs::read_dir(fd_path) {
                                    for fd_entry in fd_entries.flatten() {
                                        if let Ok(link_target) = std::fs::read_link(fd_entry.path())
                                        {
                                            if link_target == shm_path {
                                                processes.push(pid);
                                                break;
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // If we found HORUS processes, return early
        if !processes.is_empty() {
            return processes;
        }
    }

    // Fallback: scan all processes but use cheap fd check before expensive maps read
    if let Ok(proc_entries) = std::fs::read_dir("/proc") {
        for entry in proc_entries.flatten() {
            let pid = match entry
                .file_name()
                .to_str()
                .and_then(|s| s.parse::<u32>().ok())
            {
                Some(p) if p >= 100 => p, // Skip kernel threads
                _ => continue,
            };

            // Cheap check first: file descriptors (readlink is faster than reading maps)
            let fd_path = entry.path().join("fd");
            if let Ok(fd_entries) = std::fs::read_dir(&fd_path) {
                let mut found = false;
                for fd_entry in fd_entries.flatten() {
                    if let Ok(link_target) = std::fs::read_link(fd_entry.path()) {
                        if link_target == shm_path {
                            processes.push(pid);
                            found = true;
                            break;
                        }
                    }
                }
                if found {
                    continue;
                }
            }

            // Expensive fallback: check memory maps for mmap'd files
            let maps_path = entry.path().join("maps");
            if let Ok(maps_content) = std::fs::read_to_string(&maps_path) {
                if maps_content.contains(&*shm_path_str) {
                    processes.push(pid);
                }
            }
        }
    }

    processes
}
