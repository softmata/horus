//! Clean command - Clean build artifacts and shared memory
//!
//! Removes build caches, shared memory, and other temporary files.

use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::path::Path;

/// Run the clean command
pub fn run_clean(shm: bool, all: bool, dry_run: bool) -> HorusResult<()> {
    println!("{}", "Cleaning HORUS artifacts...".cyan().bold());
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
            println!(
                "{} Would clean the above items. Run without --dry-run to apply.",
                "".yellow()
            );
        } else {
            println!("{} Clean complete!", "".green());
        }
    } else {
        println!("{} Nothing to clean.", "".dimmed());
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
                "".cyan(),
                "target/".white(),
                format_size(size)
            );
        } else {
            println!(
                "  {} Removing {} ({})",
                "".cyan(),
                "target/".white(),
                format_size(size)
            );
            std::fs::remove_dir_all(target_dir).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        println!("  {} No target/ directory found", "".dimmed());
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
                "".cyan(),
                shm_base.white(),
                format_size(size),
                file_count
            );
        } else {
            println!(
                "  {} Removing {} ({}, {} files)",
                "".cyan(),
                shm_base.white(),
                format_size(size),
                file_count
            );
            std::fs::remove_dir_all(shm_path).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        println!("  {} No shared memory at {}", "".dimmed(), shm_base);
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
                "".cyan(),
                format_size(size),
                file_count
            );
        } else {
            println!(
                "  {} Removing ~/.horus/cache/ ({}, {} files)",
                "".cyan(),
                format_size(size),
                file_count
            );
            std::fs::remove_dir_all(&cache_dir).map_err(HorusError::Io)?;
        }
        return Ok(true);
    } else {
        println!("  {} No cache at ~/.horus/cache/", "".dimmed());
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
    if bytes < 1024 {
        format!("{} B", bytes)
    } else if bytes < 1024 * 1024 {
        format!("{:.1} KB", bytes as f64 / 1024.0)
    } else if bytes < 1024 * 1024 * 1024 {
        format!("{:.1} MB", bytes as f64 / (1024.0 * 1024.0))
    } else {
        format!("{:.1} GB", bytes as f64 / (1024.0 * 1024.0 * 1024.0))
    }
}
