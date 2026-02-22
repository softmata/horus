use super::*;

// Unified backend functions using monitor module

pub(super) fn get_active_nodes() -> Result<Vec<NodeStatus>> {
    // Use unified backend from monitor module
    let discovered_nodes = crate::discovery::discover_nodes().unwrap_or_default();

    if discovered_nodes.is_empty() {
        // Show demo data if no real nodes detected
        Ok(vec![NodeStatus {
            name: "No HORUS nodes detected".to_string(),
            status: "inactive".to_string(),
            cpu_usage: 0.0,
            memory_usage: 0,
            process_id: 0,
            priority: 0,
            publishers: Vec::new(),
            subscribers: Vec::new(),
        }])
    } else {
        Ok(discovered_nodes
            .into_iter()
            .map(|n| NodeStatus {
                name: n.name.clone(),
                status: if n.status == "Running" {
                    "active".to_string()
                } else {
                    "inactive".to_string()
                },
                cpu_usage: n.cpu_usage,
                memory_usage: n.memory_usage,
                process_id: n.process_id,
                priority: n.priority,
                publishers: n.publishers.iter().map(|p| p.topic.clone()).collect(),
                subscribers: n.subscribers.iter().map(|s| s.topic.clone()).collect(),
            })
            .collect())
    }
}

pub(super) fn get_local_workspaces(current_workspace_path: &Option<std::path::PathBuf>) -> Vec<WorkspaceData> {
    use std::collections::HashSet;
    use std::fs;

    let mut workspaces = Vec::new();

    // Use unified workspace discovery
    let discovered = crate::workspace::discover_all_workspaces(current_workspace_path);

    for ws in discovered {
        let env_path_buf = ws.path;
        let horus_dir = env_path_buf.join(".horus");

        // Read dependencies from horus.yaml
        let horus_yaml_path = env_path_buf.join("horus.yaml");
        let yaml_dependencies = if horus_yaml_path.exists() {
            fs::read_to_string(&horus_yaml_path)
                .ok()
                .and_then(|content| serde_yaml::from_str::<serde_yaml::Value>(&content).ok())
                .and_then(|yaml| {
                    yaml.get("dependencies")
                        .and_then(|deps| deps.as_sequence())
                        .map(|seq| {
                            seq.iter()
                                .filter_map(|v| v.as_str().map(|s| s.to_string()))
                                .collect::<Vec<String>>()
                        })
                })
                .unwrap_or_default()
        } else {
            Vec::new()
        };

        // Get packages inside this workspace
        let packages_dir = horus_dir.join("packages");
        let mut packages = Vec::new();
        let mut installed_packages_set: HashSet<String> = HashSet::new();

        if packages_dir.exists() {
            if let Ok(pkg_entries) = fs::read_dir(&packages_dir) {
                for pkg_entry in pkg_entries.flatten() {
                    // Check if it's a directory OR a symlink (include broken symlinks to show them)
                    let file_type = pkg_entry.file_type().ok();
                    let is_dir = file_type.as_ref().map(|t| t.is_dir()).unwrap_or(false);
                    let is_symlink = file_type.as_ref().map(|t| t.is_symlink()).unwrap_or(false);
                    let is_pkg_entry = is_dir || is_symlink;

                    if is_pkg_entry {
                        let pkg_name = pkg_entry.file_name().to_string_lossy().to_string();
                        let pkg_path = pkg_entry.path();

                        // Check if symlink target exists (for broken symlink detection)
                        let symlink_broken = is_symlink && !pkg_path.exists();

                        // Try to get version from metadata.json (follow symlinks if valid)
                        let metadata_path = pkg_path.join("metadata.json");
                        let version = if symlink_broken {
                            "broken link".to_string()
                        } else if metadata_path.exists() {
                            fs::read_to_string(&metadata_path)
                                .ok()
                                .and_then(|s| serde_json::from_str::<serde_json::Value>(&s).ok())
                                .and_then(|j| {
                                    j.get("version")
                                        .and_then(|v| v.as_str())
                                        .map(|s| s.to_string())
                                })
                                .unwrap_or_else(|| "unknown".to_string())
                        } else {
                            "unknown".to_string()
                        };

                        // Scan for installed packages inside this package's .horus/packages/
                        let nested_packages_dir = pkg_entry.path().join(".horus/packages");
                        let mut installed_packages = Vec::new();

                        if nested_packages_dir.exists() {
                            if let Ok(nested_entries) = fs::read_dir(&nested_packages_dir) {
                                for nested_entry in nested_entries.flatten() {
                                    if nested_entry
                                        .file_type()
                                        .map(|t| t.is_dir())
                                        .unwrap_or(false)
                                    {
                                        let nested_name =
                                            nested_entry.file_name().to_string_lossy().to_string();

                                        // Try to get version
                                        let nested_metadata_path =
                                            nested_entry.path().join("metadata.json");
                                        let nested_version = if nested_metadata_path.exists() {
                                            fs::read_to_string(&nested_metadata_path)
                                                .ok()
                                                .and_then(|s| {
                                                    serde_json::from_str::<serde_json::Value>(&s)
                                                        .ok()
                                                })
                                                .and_then(|j| {
                                                    j.get("version")
                                                        .and_then(|v| v.as_str())
                                                        .map(|s| s.to_string())
                                                })
                                                .unwrap_or_else(|| "unknown".to_string())
                                        } else {
                                            "unknown".to_string()
                                        };

                                        installed_packages.push((nested_name, nested_version));
                                    }
                                }
                            }
                        }

                        installed_packages_set.insert(pkg_name.clone());
                        packages.push(PackageData {
                            name: pkg_name,
                            version,
                            installed_packages,
                        });
                    }
                }
            }
        }

        // Process dependencies from horus.yaml - only include those NOT already installed
        let dependencies: Vec<DependencyData> = yaml_dependencies
            .iter()
            .filter_map(|dep_str| {
                let dep_name = dep_str.split('@').next().unwrap_or(dep_str);

                // Skip if already in installed packages
                if installed_packages_set.contains(dep_name) {
                    return None;
                }

                // This dependency is declared but not installed
                Some(DependencyData {
                    name: dep_name.to_string(),
                    declared_version: dep_str.clone(),
                    _status: DependencyStatus::Missing,
                })
            })
            .collect();

        // Always add the workspace, even if it has no packages
        workspaces.push(WorkspaceData {
            name: ws.name,
            path: env_path_buf.to_string_lossy().to_string(),
            packages,
            dependencies,
            is_current: ws.is_current,
        });
    }

    // Sort by: current workspace first, then alphabetically
    workspaces.sort_by(|a, b| {
        match (a.is_current, b.is_current) {
            (true, false) => std::cmp::Ordering::Less, // Current workspace comes first
            (false, true) => std::cmp::Ordering::Greater, // Current workspace comes first
            _ => a.name.cmp(&b.name),                  // Otherwise sort alphabetically
        }
    });
    workspaces
}

type PackageInfo = (String, String, String);
type InstalledPackages = (Vec<PackageInfo>, Vec<PackageInfo>);

/// Recursively calculate total size of a directory
fn calculate_dir_size(path: &std::path::Path) -> u64 {
    let mut total = 0;

    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            if let Ok(metadata) = entry.metadata() {
                if metadata.is_file() {
                    total += metadata.len();
                } else if metadata.is_dir() {
                    total += calculate_dir_size(&entry.path());
                }
            }
        }
    }

    total
}

/// Read version from package metadata.json
fn get_package_version(pkg_path: &std::path::Path) -> String {
    let metadata_path = pkg_path.join("metadata.json");

    if let Ok(content) = std::fs::read_to_string(&metadata_path) {
        if let Ok(json) = serde_json::from_str::<serde_json::Value>(&content) {
            if let Some(version) = json.get("version").and_then(|v| v.as_str()) {
                return version.to_string();
            }
        }
    }

    "unknown".to_string()
}

pub(super) fn get_installed_packages() -> InstalledPackages {
    let mut local_packages = Vec::new();
    let mut global_packages = Vec::new();
    let mut seen = std::collections::HashSet::new();

    // Check local .horus/packages first (project-specific installed packages)
    let local_packages_dir = std::env::current_dir()
        .ok()
        .map(|d| d.join(".horus/packages"));

    if let Some(ref packages_dir) = local_packages_dir {
        if packages_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(packages_dir) {
                for entry in entries.flatten() {
                    if let Some(name) = entry.file_name().to_str() {
                        if seen.insert(name.to_string()) {
                            let pkg_path = entry.path();

                            // Calculate real directory size
                            let total_bytes = calculate_dir_size(&pkg_path);
                            let size = if total_bytes == 0 {
                                "Unknown".to_string()
                            } else {
                                let kb = total_bytes / 1024;
                                if kb < 1024 {
                                    format!("{} KB", kb)
                                } else {
                                    format!("{:.1} MB", kb as f64 / 1024.0)
                                }
                            };

                            // Read real version from metadata.json
                            let version = get_package_version(&pkg_path);

                            local_packages.push((name.to_string(), version, size));
                        }
                    }
                }
            }
        }
    }

    // Check global ~/.horus/cache (system-wide)
    let global_cache = dirs::home_dir().map(|h| h.join(".horus/cache"));

    if let Some(ref cache_dir) = global_cache {
        if cache_dir.exists() {
            if let Ok(entries) = std::fs::read_dir(cache_dir) {
                for entry in entries.flatten() {
                    if let Some(name) = entry.file_name().to_str() {
                        if seen.insert(name.to_string()) {
                            let pkg_path = entry.path();

                            // Calculate real directory size
                            let total_bytes = calculate_dir_size(&pkg_path);
                            let size = if total_bytes == 0 {
                                "Unknown".to_string()
                            } else {
                                let kb = total_bytes / 1024;
                                if kb < 1024 {
                                    format!("{} KB", kb)
                                } else {
                                    format!("{:.1} MB", kb as f64 / 1024.0)
                                }
                            };

                            // Read real version from metadata.json
                            let version = get_package_version(&pkg_path);

                            global_packages.push((name.to_string(), version, size));
                        }
                    }
                }
            }
        }
    }

    // Sort both lists
    local_packages.sort_by(|a, b| a.0.cmp(&b.0));
    global_packages.sort_by(|a, b| a.0.cmp(&b.0));

    // Add placeholder if both are empty
    if local_packages.is_empty() && global_packages.is_empty() {
        local_packages.push((
            "No packages found".to_string(),
            "-".to_string(),
            "-".to_string(),
        ));
    }

    (local_packages, global_packages)
}

// Removed: get_runtime_parameters() - now using real RuntimeParams from horus_core

/// Format bytes into human-readable string (B, KB, MB, GB)
pub(super) fn format_bytes(bytes: u64) -> String {
    const KB: u64 = 1024;
    const MB: u64 = KB * 1024;
    const GB: u64 = MB * 1024;

    if bytes >= GB {
        format!("{:.1}GB", bytes as f64 / GB as f64)
    } else if bytes >= MB {
        format!("{:.1}MB", bytes as f64 / MB as f64)
    } else if bytes >= KB {
        format!("{:.1}KB", bytes as f64 / KB as f64)
    } else {
        format!("{}B", bytes)
    }
}

pub(super) fn get_active_topics() -> Result<Vec<TopicInfo>> {
    // Use unified backend from monitor module
    let discovered_topics = crate::discovery::discover_shared_memory().unwrap_or_default();

    // Filter to only show active topics (ROS-like behavior)
    // Stale topics (no live processes, old modification time) are hidden
    let active_topics: Vec<_> = discovered_topics
        .into_iter()
        .filter(|t| t.status != crate::discovery::TopicStatus::Stale)
        .collect();

    if active_topics.is_empty() {
        // Return empty - no placeholder needed (clean like ROS)
        Ok(Vec::new())
    } else {
        Ok(active_topics
            .into_iter()
            .map(|t| {
                // Shorten type names for readability
                let short_type = t
                    .message_type
                    .as_ref()
                    .map(|ty| ty.split("::").last().unwrap_or(ty).to_string())
                    .unwrap_or_else(|| "Unknown".to_string());

                TopicInfo {
                    name: t.topic_name,
                    msg_type: short_type,
                    publishers: t.publishers.len(),
                    subscribers: t.subscribers.len(),
                    rate: t.message_rate_hz,
                    publisher_nodes: t.publishers,
                    subscriber_nodes: t.subscribers,
                    status: t.status,
                }
            })
            .collect())
    }
}
