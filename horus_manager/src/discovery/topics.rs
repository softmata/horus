use super::nodes::{discover_registry_files, process_exists};
use super::*;

use std::collections::HashMap as StdHashMap;

lazy_static::lazy_static! {
    static ref TOPIC_RATE_CACHE: Arc<RwLock<StdHashMap<String, (Instant, u64)>>> =
        Arc::new(RwLock::new(StdHashMap::new()));
}

pub(super) fn discover_shared_memory_uncached() -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();
    let mut seen_topics: std::collections::HashSet<String> = std::collections::HashSet::new();

    // PRIMARY: Scan SHM topics directory (has full metadata: size, mtime, accessing processes)
    let topics_path = shm_topics_dir();
    if topics_path.exists() {
        for topic in scan_topics_directory(&topics_path)? {
            seen_topics.insert(topic.topic_name.clone());
            topics.push(topic);
        }
    }

    // SUPPLEMENT: Add topics from presence files (captures network topics too)
    // These won't have SHM-specific metadata but will show pub/sub relationships
    for topic in discover_topics_from_presence() {
        // Check for duplicates - match exact name OR with horus_topic/ prefix
        let is_duplicate = seen_topics.contains(&topic.topic_name)
            || seen_topics.contains(&format!("horus_topic/{}", topic.topic_name))
            || topic
                .topic_name
                .strip_prefix("horus_topic/")
                .map(|base| seen_topics.contains(base))
                .unwrap_or(false);

        if !is_duplicate {
            seen_topics.insert(topic.topic_name.clone());
            topics.push(topic);
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

    for presence in NodePresence::read_all() {
        // Add publishers
        for pub_topic in &presence.publishers {
            let entry = topic_map
                .entry(pub_topic.topic_name.clone())
                .or_insert_with(|| (Vec::new(), Vec::new(), None));
            if !entry.0.contains(&presence.name) {
                entry.0.push(presence.name.clone());
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
        for sub_topic in &presence.subscribers {
            let entry = topic_map
                .entry(sub_topic.topic_name.clone())
                .or_insert_with(|| (Vec::new(), Vec::new(), None));
            if !entry.1.contains(&presence.name) {
                entry.1.push(presence.name.clone());
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
            }
        })
        .collect()
}

/// Clean up stale topic files from the global topics directory.
///
/// A topic is considered stale if:
/// 1. No process has it mmap'd (no live readers/writers)
/// 2. It hasn't been modified in 5+ minutes
///
/// Internal: Clean up stale topic files in a specific directory
pub(super) fn cleanup_stale_topics_in_dir(shm_path: &Path) {
    const STALE_THRESHOLD_SECS: u64 = 300; // 5 minutes

    if let Ok(entries) = std::fs::read_dir(shm_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if !path.is_file() {
                continue;
            }

            let Some(name) = path.file_name().and_then(|s| s.to_str()) else {
                continue;
            };

            // Check if any process has this file mmap'd
            let accessing_procs = find_accessing_processes_fast(&path, name);
            let has_live_processes = accessing_procs.iter().any(|pid| process_exists(*pid));

            if has_live_processes {
                continue; // Topic is in use
            }

            // Check modification time
            if let Ok(metadata) = entry.metadata() {
                if let Ok(modified) = metadata.modified() {
                    if let Ok(elapsed) = modified.elapsed() {
                        if elapsed.as_secs() > STALE_THRESHOLD_SECS {
                            // Topic is stale - remove it
                            let _ = std::fs::remove_file(&path);
                        }
                    }
                }
            }
        }
    }
}

/// Scan a specific topics directory for shared memory files
fn scan_topics_directory(shm_path: &Path) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    // Load registry to get topic metadata
    let registry_topics = load_topic_metadata_from_registry();

    // Active nodes for pub/sub inference (from registry)
    let active_nodes: Vec<String> = Vec::new();

    for entry in std::fs::read_dir(shm_path)? {
        let entry = entry?;
        let path = entry.path();
        let metadata = entry.metadata()?;

        // Smart filter for shared memory segments
        if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
            if metadata.is_file() {
                // Hub topics - files directly in topics directory
                if let Some(info) = scan_topic_file(&path, name, &registry_topics, &active_nodes) {
                    topics.push(info);
                }
            } else if metadata.is_dir() && name == "horus_links" {
                // Link topics - files inside horus_links subdirectory
                topics.extend(scan_links_directory(
                    &path,
                    &registry_topics,
                    &active_nodes,
                )?);
            } else if metadata.is_dir() && name == "horus_topic" {
                // Topic API topics - files inside horus_topic subdirectory
                // These are created by Topic::new() in HORUS applications
                topics.extend(scan_topic_api_directory(
                    &path,
                    &registry_topics,
                    &active_nodes,
                )?);
            }
        }
    }

    Ok(topics)
}

/// Scan the horus_links directory for Link shared memory files
fn scan_links_directory(
    links_path: &Path,
    registry_topics: &StdHashMap<String, (String, Vec<String>, Vec<String>)>,
    active_nodes: &[String],
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
                            scan_topic_file(&path, name, registry_topics, active_nodes)
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
    active_nodes: &[String],
) -> HorusResult<Vec<SharedMemoryInfo>> {
    let mut topics = Vec::new();

    if let Ok(entries) = std::fs::read_dir(topic_path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    if let Some(name) = path.file_name().and_then(|s| s.to_str()) {
                        // Skip discovery metadata files
                        if name == "horus.discovery" {
                            continue;
                        }
                        // Topic API uses horus_topic/<name> path format
                        // Include the subdirectory in topic_name for proper path construction
                        let topic_name = format!("horus_topic/{}", name);
                        if let Some(mut info) =
                            scan_topic_file(&path, name, registry_topics, active_nodes)
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
    active_nodes: &[String],
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
        mod_time.elapsed().unwrap_or(Duration::from_secs(3600)) < Duration::from_secs(30)
    } else {
        false
    };

    let has_valid_processes = accessing_procs.iter().any(|pid| process_exists(*pid));

    // Include all topics in HORUS directory
    // Topics persist until manually cleaned or processes terminate
    let active = has_valid_processes || is_recent;

    // Calculate message rate from modification times
    let message_rate = calculate_topic_rate(&topic_name, modified);

    // Get metadata from registry
    let (message_type, mut publishers, subscribers) = registry_topics
        .get(&topic_name)
        .map(|(t, p, s)| (Some(t.clone()), p.clone(), s.clone()))
        .unwrap_or((None, Vec::new(), Vec::new()));

    // Fallback: if no registry info and topic is active, infer from active nodes
    if publishers.is_empty() && active && !active_nodes.is_empty() {
        // Assume all active nodes are potential publishers for active topics
        // This provides visibility when registry.json is not available
        publishers = active_nodes.to_vec();
    }

    // Compute live processes (filter dead ones)
    let live_processes: Vec<u32> = accessing_procs
        .iter()
        .filter(|pid| process_exists(**pid))
        .copied()
        .collect();

    // Compute status and age string
    let (status, age_string) = compute_topic_status(&live_processes, modified);

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
    })
}

/// Compute topic status and age string based on live processes and modification time
fn compute_topic_status(
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
fn format_age(secs: u64) -> String {
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

fn calculate_topic_rate(topic_name: &str, modified: Option<std::time::SystemTime>) -> f32 {
    let now = Instant::now();

    if let Some(mod_time) = modified {
        if let Ok(mut cache) = TOPIC_RATE_CACHE.write() {
            // Convert SystemTime to a simple counter for change detection
            let mod_counter = mod_time
                .duration_since(std::time::UNIX_EPOCH)
                .map(|d| d.as_millis() as u64)
                .unwrap_or(0);

            if let Some((prev_instant, prev_counter)) = cache.get(topic_name) {
                if mod_counter != *prev_counter {
                    // File was modified
                    let time_delta = now.duration_since(*prev_instant).as_secs_f32();
                    if time_delta > 0.0 && time_delta < 10.0 {
                        let rate = 1.0 / time_delta;
                        cache.insert(topic_name.to_string(), (now, mod_counter));
                        return rate;
                    }
                }
            }

            // First sample or same modification time
            cache.insert(topic_name.to_string(), (now, mod_counter));
        }
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
                            if cmdline_str.contains("horus")
                                || cmdline_str.contains("ros")
                                || cmdline_str.contains("sim")
                                || cmdline_str.contains("snake")
                            {
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

    // Fallback: Abbreviated scan - only check first 50 processes to avoid blocking
    if let Ok(proc_entries) = std::fs::read_dir("/proc") {
        for (_checked, entry) in proc_entries.flatten().enumerate().take(50) {
            if let Some(pid) = entry
                .file_name()
                .to_str()
                .and_then(|s| s.parse::<u32>().ok())
            {
                // Check memory maps first (mmap'd files)
                let maps_path = entry.path().join("maps");
                if let Ok(maps_content) = std::fs::read_to_string(&maps_path) {
                    if maps_content.contains(&*shm_path_str) {
                        processes.push(pid);
                        continue;
                    }
                }
                // Fallback to file descriptors
                let fd_path = entry.path().join("fd");
                if let Ok(fd_entries) = std::fs::read_dir(fd_path) {
                    for fd_entry in fd_entries.flatten() {
                        if let Ok(link_target) = std::fs::read_link(fd_entry.path()) {
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

    processes
}
