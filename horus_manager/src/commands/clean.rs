//! Clean command - Clean build artifacts and shared memory
//!
//! Removes build caches, shared memory, and other temporary files.

use crate::cli_output;
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
            let cpp_build_dir = std::path::Path::new(".horus/cpp-build");
            if cpp_build_dir.exists() {
                items.push(serde_json::json!({
                    "type": "cpp_build_cache",
                    "path": ".horus/cpp-build/",
                    "size": get_dir_size(cpp_build_dir),
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
            // Perform cleaning silently — JSON output was already emitted above.
            // Suppress human-readable print calls from sub-functions.
            let was_quiet = crate::progress::is_quiet();
            crate::progress::set_quiet(true);
            let result = (|| -> HorusResult<()> {
                if !shm || all {
                    clean_build_cache(false)?;
                    clean_cpp_build(false)?;
                    clean_pycache(false)?;
                }
                if shm || all {
                    clean_shared_memory(false, force)?;
                }
                if all {
                    clean_horus_cache(false)?;
                }
                Ok(())
            })();
            crate::progress::set_quiet(was_quiet);
            result?;
        }
        return Ok(());
    }

    cli_output::header("Cleaning HORUS artifacts...");
    if !crate::progress::is_quiet() {
        println!();
    }

    let mut cleaned_anything = false;

    // Clean build cache (target directory)
    if !shm || all {
        cleaned_anything |= clean_build_cache(dry_run)?;
    }

    // Clean C++ build artifacts
    if !shm || all {
        cleaned_anything |= clean_cpp_build(dry_run)?;
    }

    // Clean Python bytecode caches
    if !shm || all {
        cleaned_anything |= clean_pycache(dry_run)?;
    }

    // Clean shared memory
    if shm || all {
        cleaned_anything |= clean_shared_memory(dry_run, force)?;
    }

    // Clean HORUS cache directory
    if all {
        cleaned_anything |= clean_horus_cache(dry_run)?;
    }

    if !crate::progress::is_quiet() {
        println!();
    }
    if cleaned_anything {
        if dry_run {
            cli_output::warn("Would clean the above items. Run without --dry-run to apply.");
        } else {
            cli_output::success("Clean complete!");
        }
    } else {
        cli_output::info("Nothing to clean.");
    }

    Ok(())
}

/// Clean build cache (target directory)
fn clean_build_cache(dry_run: bool) -> HorusResult<bool> {
    let target_dir = Path::new("target");

    if target_dir.exists() {
        let size = get_dir_size(target_dir);

        if dry_run {
            cli_output::info(&format!(
                "Would remove {} ({})",
                "target/".white(),
                format_size(size)
            ));
        } else {
            cli_output::info(&format!(
                "Removing {} ({})",
                "target/".white(),
                format_size(size)
            ));
            std::fs::remove_dir_all(target_dir).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        cli_output::info("No target/ directory found");
    }

    Ok(false)
}

/// Clean shared memory — scans ALL horus_* namespace directories
fn clean_shared_memory(dry_run: bool, force: bool) -> HorusResult<bool> {
    let namespaces = list_all_horus_namespaces();

    if namespaces.is_empty() {
        cli_output::info("No HORUS shared memory namespaces found");
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
                    node.name(),
                    node.pid()
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
    let cache_dir = crate::paths::cache_dir()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

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

/// Clean C++ cmake build artifacts (.horus/cpp-build/)
fn clean_cpp_build(dry_run: bool) -> HorusResult<bool> {
    let cpp_build_dir = Path::new(".horus/cpp-build");

    if cpp_build_dir.exists() {
        let size = get_dir_size(cpp_build_dir);

        if dry_run {
            cli_output::info(&format!(
                "Would remove {} ({})",
                ".horus/cpp-build/".white(),
                format_size(size)
            ));
        } else {
            cli_output::info(&format!(
                "Removing {} ({})",
                ".horus/cpp-build/".white(),
                format_size(size)
            ));
            std::fs::remove_dir_all(cpp_build_dir).map_err(HorusError::Io)?;
        }
        return Ok(true);
    }

    Ok(false)
}

/// Clean Python __pycache__/ directories recursively from the project
fn clean_pycache(dry_run: bool) -> HorusResult<bool> {
    let current = Path::new(".");
    let pycache_dirs = find_pycache_dirs(current);

    if pycache_dirs.is_empty() {
        return Ok(false);
    }

    let total_size: u64 = pycache_dirs.iter().map(|d| get_dir_size(d)).sum();

    if dry_run {
        cli_output::info(&format!(
            "Would remove {} __pycache__/ directories ({})",
            pycache_dirs.len(),
            format_size(total_size)
        ));
    } else {
        for dir in &pycache_dirs {
            let _ = std::fs::remove_dir_all(dir);
        }
        cli_output::info(&format!(
            "Removed {} __pycache__/ directories ({})",
            pycache_dirs.len(),
            format_size(total_size)
        ));
    }

    Ok(true)
}

/// Recursively find all __pycache__/ directories, skipping .horus/ and target/
fn find_pycache_dirs(root: &Path) -> Vec<std::path::PathBuf> {
    let mut result = Vec::new();
    if let Ok(entries) = std::fs::read_dir(root) {
        for entry in entries.flatten() {
            let path = entry.path();
            if path.is_dir() {
                let name = entry.file_name();
                let name_str = name.to_string_lossy();
                // Skip build artifact directories to avoid deep traversals
                if name_str == "target" || name_str == ".horus" || name_str == ".git" {
                    continue;
                }
                if name_str == "__pycache__" {
                    result.push(path);
                } else {
                    result.extend(find_pycache_dirs(&path));
                }
            }
        }
    }
    result
}

use crate::fs_utils;

fn get_dir_size(path: &Path) -> u64 {
    fs_utils::dir_size(path)
}

fn count_files(path: &Path) -> usize {
    fs_utils::count_files(path)
}

fn format_size(bytes: u64) -> String {
    fs_utils::format_bytes(bytes)
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
        assert!(
            !result.is_empty(),
            "format_size should return non-empty string"
        );
        // 1024 bytes should format as some form of "1 KB" or "1.0 KB"
        assert!(
            result.contains("KB") || result.contains("KiB") || result.contains("1"),
            "1024 bytes should include KB or 1 in output, got: {}",
            result
        );
        // Zero bytes should still produce output
        let zero = format_size(0);
        assert!(
            !zero.is_empty(),
            "format_size(0) should return non-empty string"
        );

        // Large sizes should produce different output than small sizes
        let large = format_size(1_000_000_000);
        assert_ne!(
            format_size(100),
            large,
            "100 bytes and 1GB should format differently"
        );
    }

    // ── Battle-testing: run_clean ────────────────────────────────────────

    #[test]
    fn battle_clean_dry_run_no_artifacts() {
        let tmp = tempfile::tempdir().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // No target/, no shm — dry run should succeed
        let result = run_clean(false, false, true, false, false);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn battle_clean_dry_run_with_target_dir() {
        let tmp = tempfile::tempdir().unwrap();
        let target = tmp.path().join("target");
        fs::create_dir(&target).unwrap();
        fs::write(target.join("dummy"), "build artifact").unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // Dry run should not delete
        let result = run_clean(false, false, true, false, false);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
        assert!(target.exists(), "dry run should NOT delete target/");
    }

    #[test]
    fn battle_clean_removes_target_dir() {
        let tmp = tempfile::tempdir().unwrap();
        let target = tmp.path().join("target");
        fs::create_dir(&target).unwrap();
        fs::write(target.join("dummy.o"), vec![0u8; 50]).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_clean(false, false, false, false, false);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
        assert!(!target.exists(), "clean should delete target/");
    }

    #[test]
    fn battle_clean_json_output() {
        let tmp = tempfile::tempdir().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // JSON mode with dry_run — should succeed and output valid JSON
        let result = run_clean(false, false, true, false, true);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn battle_clean_shm_only_flag() {
        let tmp = tempfile::tempdir().unwrap();
        let target = tmp.path().join("target");
        fs::create_dir(&target).unwrap();
        fs::write(target.join("dummy"), "keep me").unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        // shm=true means only clean shared memory, not target/
        let result = run_clean(true, false, false, false, false);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
        assert!(target.exists(), "shm-only clean should NOT touch target/");
    }

    #[test]
    fn battle_clean_build_cache_empty_target() {
        let tmp = tempfile::tempdir().unwrap();
        let target = tmp.path().join("target");
        fs::create_dir(&target).unwrap();
        // Empty target dir

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = clean_build_cache(false);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
        assert!(result.unwrap()); // Returns true because target existed
        assert!(!target.exists());
    }

    #[test]
    fn battle_clean_build_cache_no_target() {
        let tmp = tempfile::tempdir().unwrap();
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = clean_build_cache(false);
        std::env::set_current_dir(&prev).unwrap();
        assert!(result.is_ok());
        assert!(!result.unwrap()); // Returns false because no target/
    }
}
