//! Clean command - Clean build artifacts and shared memory
//!
//! Removes build caches, shared memory, and other temporary files.

use crate::cli_output;
use crate::progress::format_bytes;
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use horus_core::memory::{list_all_horus_namespaces, shm_namespace, NamespaceInfo};
use horus_core::NodePresence;
use std::path::Path;

/// Run the clean command
pub fn run_clean(shm: bool, all: bool, dry_run: bool, force: bool, json: bool) -> HorusResult<()> {
    if json {
        let mut items = Vec::new();
        if !shm || all {
            let target_dir = std::path::Path::new("target");
            if target_dir.exists() {
                items.push(serde_json::json!({
                    "type": "build_cache",
                    "path": "target/",
                    "size": get_dir_size(target_dir),
                    "exists": true,
                }));
            }
        }
        if shm || all {
            let namespaces = list_all_horus_namespaces();
            for ns in &namespaces {
                items.push(serde_json::json!({
                    "type": "shared_memory",
                    "path": ns.path.display().to_string(),
                    "namespace": ns.dir_name,
                    "size": ns.size_bytes,
                    "files": ns.file_count,
                    "alive": ns.alive,
                    "exists": true,
                }));
            }
        }
        if all {
            if let Ok(cache_dir) = crate::paths::cache_dir() {
                if cache_dir.exists() {
                    items.push(serde_json::json!({
                        "type": "horus_cache",
                        "path": "~/.horus/cache/",
                        "size": get_dir_size(&cache_dir),
                        "files": count_files(&cache_dir),
                        "exists": true,
                    }));
                }
            }
        }
        let output = serde_json::json!({
            "dry_run": dry_run,
            "items": items,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        if !dry_run {
            // Actually perform the cleaning
            if !shm || all {
                clean_build_cache(false)?;
            }
            if shm || all {
                clean_shared_memory(false, force)?;
            }
            if all {
                clean_horus_cache(false)?;
            }
        }
        return Ok(());
    }

    cli_output::header("Cleaning HORUS artifacts...");
    println!();

    let mut cleaned_anything = false;

    // Clean build cache (target directory)
    if !shm || all {
        cleaned_anything |= clean_build_cache(dry_run)?;
    }

    // Clean shared memory
    if shm || all {
        cleaned_anything |= clean_shared_memory(dry_run, force)?;
    }

    // Clean HORUS cache directory
    if all {
        cleaned_anything |= clean_horus_cache(dry_run)?;
    }

    println!();
    if cleaned_anything {
        if dry_run {
            cli_output::warn("Would clean the above items. Run without --dry-run to apply.");
        } else {
            cli_output::success("Clean complete!");
        }
    } else {
        println!("{} Nothing to clean.", cli_output::ICON_SUCCESS.dimmed());
    }

    Ok(())
}

/// Clean build cache (target directory)
fn clean_build_cache(dry_run: bool) -> HorusResult<bool> {
    let target_dir = Path::new("target");

    if target_dir.exists() {
        let size = get_dir_size(target_dir);

        if dry_run {
            println!(
                "  {} Would remove {} ({})",
                cli_output::ICON_INFO.cyan(),
                "target/".white(),
                format_size(size)
            );
        } else {
            println!(
                "  {} Removing {} ({})",
                cli_output::ICON_INFO.cyan(),
                "target/".white(),
                format_size(size)
            );
            std::fs::remove_dir_all(target_dir).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        println!(
            "  {} No target/ directory found",
            cli_output::ICON_SUCCESS.dimmed()
        );
    }

    Ok(false)
}

/// Clean shared memory — scans ALL horus_* namespace directories
fn clean_shared_memory(dry_run: bool, force: bool) -> HorusResult<bool> {
    let namespaces = list_all_horus_namespaces();

    if namespaces.is_empty() {
        println!(
            "  {} No HORUS shared memory namespaces found",
            cli_output::ICON_SUCCESS.dimmed()
        );
        return Ok(false);
    }

    let current_ns = format!("horus_{}", shm_namespace());

    // Categorize namespaces
    let mut stale: Vec<&NamespaceInfo> = Vec::new();
    let mut active: Vec<&NamespaceInfo> = Vec::new();
    for ns in &namespaces {
        if ns.alive {
            active.push(ns);
        } else {
            stale.push(ns);
        }
    }

    // Display namespace table
    println!(
        "  {} Found {} HORUS namespace(s):",
        cli_output::ICON_INFO.cyan(),
        namespaces.len()
    );
    println!();
    println!(
        "    {:<40} {:>10} {:>6}  {}",
        "Namespace".white().bold(),
        "Size".white().bold(),
        "Files".white().bold(),
        "Status".white().bold()
    );
    println!("    {}", "-".repeat(72).dimmed());

    for ns in &namespaces {
        let status = if ns.dir_name == current_ns {
            "current".green().to_string()
        } else if ns.alive {
            "active".yellow().to_string()
        } else {
            "stale".red().to_string()
        };

        let name_display = if ns.dir_name == current_ns {
            format!("{} {}", ns.dir_name, "(this)".dimmed())
        } else {
            ns.dir_name.clone()
        };

        println!(
            "    {:<40} {:>10} {:>6}  {}",
            name_display,
            format_size(ns.size_bytes),
            ns.file_count,
            status
        );
    }
    println!();

    if stale.is_empty() && !force {
        println!(
            "  {} No stale namespaces to clean.",
            cli_output::ICON_SUCCESS.dimmed()
        );
        if !active.is_empty() {
            println!(
                "  {} Use {} to also remove active namespaces.",
                "Tip:".dimmed(),
                "--force".white().bold()
            );
        }
        return Ok(false);
    }

    // Determine which namespaces to remove
    let to_remove: Vec<&NamespaceInfo> = if force {
        // With --force: remove all namespaces (stale + active, including current)
        namespaces.iter().collect()
    } else {
        // Default: remove only stale namespaces
        stale
    };

    if to_remove.is_empty() {
        return Ok(false);
    }

    // Check for active nodes if forcing removal of active namespaces
    if force {
        let active_nodes = NodePresence::read_all();
        if !active_nodes.is_empty() {
            println!(
                "  {} Forcing SHM clean with {} active process(es):",
                cli_output::ICON_WARN.yellow(),
                active_nodes.len()
            );
            for node in &active_nodes {
                println!(
                    "    {} {} (PID {})",
                    cli_output::ICON_INFO.cyan(),
                    node.name,
                    node.pid
                );
            }
            println!();
        }
    }

    let total_size: u64 = to_remove.iter().map(|ns| ns.size_bytes).sum();
    let total_files: usize = to_remove.iter().map(|ns| ns.file_count).sum();

    if dry_run {
        println!(
            "  {} Would remove {} namespace(s) ({}, {} files)",
            cli_output::ICON_INFO.cyan(),
            to_remove.len(),
            format_size(total_size),
            total_files
        );
    } else {
        let mut removed = 0usize;
        let mut freed = 0u64;
        for ns in &to_remove {
            match std::fs::remove_dir_all(&ns.path) {
                Ok(()) => {
                    removed += 1;
                    freed += ns.size_bytes;
                    println!(
                        "  {} Removed {} ({})",
                        cli_output::ICON_INFO.cyan(),
                        ns.dir_name.white(),
                        format_size(ns.size_bytes)
                    );
                }
                Err(e) => {
                    println!(
                        "  {} Failed to remove {}: {}",
                        cli_output::ICON_WARN.yellow(),
                        ns.dir_name,
                        e
                    );
                }
            }
        }
        println!();
        println!(
            "  {} Removed {} namespace(s), freed {}",
            cli_output::ICON_SUCCESS.green(),
            removed,
            format_size(freed)
        );
    }

    Ok(true)
}

/// Clean HORUS cache directory
fn clean_horus_cache(dry_run: bool) -> HorusResult<bool> {
    let cache_dir = crate::paths::cache_dir().map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if cache_dir.exists() {
        let size = get_dir_size(&cache_dir);
        let file_count = count_files(&cache_dir);

        if dry_run {
            println!(
                "  {} Would remove ~/.horus/cache/ ({}, {} files)",
                cli_output::ICON_INFO.cyan(),
                format_size(size),
                file_count
            );
        } else {
            println!(
                "  {} Removing ~/.horus/cache/ ({}, {} files)",
                cli_output::ICON_INFO.cyan(),
                format_size(size),
                file_count
            );
            std::fs::remove_dir_all(&cache_dir).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        println!(
            "  {} No cache at ~/.horus/cache/",
            cli_output::ICON_SUCCESS.dimmed()
        );
    }

    Ok(false)
}

/// Get total size of directory recursively
fn get_dir_size(path: &Path) -> u64 {
    let mut size = 0;
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_file() {
                if let Ok(meta) = path.metadata() {
                    size += meta.len();
                }
            } else if path.is_dir() {
                size += get_dir_size(&path);
            }
        }
    }
    size
}

/// Count files in directory recursively
fn count_files(path: &Path) -> usize {
    let mut count = 0;
    if let Ok(entries) = std::fs::read_dir(path) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_file() {
                count += 1;
            } else if path.is_dir() {
                count += count_files(&path);
            }
        }
    }
    count
}

/// Format byte size for display
fn format_size(bytes: u64) -> String {
    format_bytes(bytes)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    #[test]
    fn get_dir_size_empty_dir() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(get_dir_size(tmp.path()), 0);
    }

    #[test]
    fn get_dir_size_with_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "hello").unwrap();
        fs::write(tmp.path().join("b.txt"), "world!").unwrap();
        let size = get_dir_size(tmp.path());
        assert_eq!(size, 11); // 5 + 6
    }

    #[test]
    fn get_dir_size_recursive() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("subdir");
        fs::create_dir(&sub).unwrap();
        fs::write(sub.join("file.bin"), vec![0u8; 100]).unwrap();
        assert_eq!(get_dir_size(tmp.path()), 100);
    }

    #[test]
    fn get_dir_size_nonexistent() {
        assert_eq!(get_dir_size(Path::new("/nonexistent/path")), 0);
    }

    #[test]
    fn count_files_empty_dir() {
        let tmp = tempfile::tempdir().unwrap();
        assert_eq!(count_files(tmp.path()), 0);
    }

    #[test]
    fn count_files_with_files() {
        let tmp = tempfile::tempdir().unwrap();
        fs::write(tmp.path().join("a.txt"), "").unwrap();
        fs::write(tmp.path().join("b.txt"), "").unwrap();
        assert_eq!(count_files(tmp.path()), 2);
    }

    #[test]
    fn count_files_recursive() {
        let tmp = tempfile::tempdir().unwrap();
        let sub = tmp.path().join("sub");
        fs::create_dir(&sub).unwrap();
        fs::write(tmp.path().join("top.txt"), "").unwrap();
        fs::write(sub.join("nested.txt"), "").unwrap();
        assert_eq!(count_files(tmp.path()), 2);
    }

    #[test]
    fn count_files_nonexistent() {
        assert_eq!(count_files(Path::new("/nonexistent")), 0);
    }

    #[test]
    fn format_size_delegates_to_format_bytes() {
        let result = format_size(1024);
        assert!(!result.is_empty());
    }
}
