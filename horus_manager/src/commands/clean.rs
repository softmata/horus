//! Clean command - Clean build artifacts and shared memory
//!
//! Removes build caches, shared memory, and other temporary files.

use crate::cli_output;
use crate::progress::format_bytes;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::path::Path;

/// Run the clean command
pub fn run_clean(shm: bool, all: bool, dry_run: bool, json: bool) -> HorusResult<()> {
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
            let shm_base = if cfg!(target_os = "macos") { "/tmp/horus" } else { "/dev/shm/horus" };
            let shm_path = std::path::Path::new(shm_base);
            if shm_path.exists() {
                items.push(serde_json::json!({
                    "type": "shared_memory",
                    "path": shm_base,
                    "size": get_dir_size(shm_path),
                    "files": count_files(shm_path),
                    "exists": true,
                }));
            }
        }
        if all {
            if let Ok(home) = dirs::home_dir().ok_or(()) {
                let cache_dir = home.join(".horus").join("cache");
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
        println!("{}", serde_json::to_string_pretty(&output).unwrap());
        if !dry_run {
            // Actually perform the cleaning
            if !shm || all { clean_build_cache(false)?; }
            if shm || all { clean_shared_memory(false)?; }
            if all { clean_horus_cache(false)?; }
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
        cleaned_anything |= clean_shared_memory(dry_run)?;
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
        println!("  {} No target/ directory found", cli_output::ICON_SUCCESS.dimmed());
    }

    Ok(false)
}

/// Clean shared memory
fn clean_shared_memory(dry_run: bool) -> HorusResult<bool> {
    let shm_base = if cfg!(target_os = "macos") {
        "/tmp/horus"
    } else {
        "/dev/shm/horus"
    };

    let shm_path = Path::new(shm_base);

    if shm_path.exists() {
        let size = get_dir_size(shm_path);
        let file_count = count_files(shm_path);

        if dry_run {
            println!(
                "  {} Would remove {} ({}, {} files)",
                cli_output::ICON_INFO.cyan(),
                shm_base.white(),
                format_size(size),
                file_count
            );
        } else {
            println!(
                "  {} Removing {} ({}, {} files)",
                cli_output::ICON_INFO.cyan(),
                shm_base.white(),
                format_size(size),
                file_count
            );
            std::fs::remove_dir_all(shm_path).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        println!("  {} No shared memory at {}", cli_output::ICON_SUCCESS.dimmed(), shm_base);
    }

    Ok(false)
}

/// Clean HORUS cache directory
fn clean_horus_cache(dry_run: bool) -> HorusResult<bool> {
    let home = dirs::home_dir()
        .ok_or_else(|| HorusError::Config("Could not determine home directory".to_string()))?;

    let cache_dir = home.join(".horus").join("cache");

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
        println!("  {} No cache at ~/.horus/cache/", cli_output::ICON_SUCCESS.dimmed());
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
