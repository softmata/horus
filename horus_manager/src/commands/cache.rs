//! Cache command - manage the HORUS package cache

use crate::progress::format_bytes;
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::fs;
use std::path::Path;

use crate::workspace;

/// Calculate the total size of a directory recursively
pub fn dir_size(path: &Path) -> std::io::Result<u64> {
    let mut size = 0;
    if path.is_dir() {
        for entry in fs::read_dir(path)? {
            let entry = entry?;
            let metadata = entry.metadata()?;
            if metadata.is_dir() {
                size += dir_size(&entry.path())?;
            } else {
                size += metadata.len();
            }
        }
    }
    Ok(size)
}

/// Format a size in bytes as a human-readable string
pub fn format_size(bytes: u64) -> String {
    format_bytes(bytes)
}

/// Show cache information (directory, size, package count)
pub fn run_info() -> HorusResult<()> {
    let cache_dir = crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

    println!("{}", "HORUS Cache Information".cyan().bold());
    println!("{}", "═".repeat(40));

    if !cache_dir.exists() {
        println!("Cache directory: {} (not created yet)", cache_dir.display());
        println!("Total size: 0 B");
        println!("Packages: 0");
        return Ok(());
    }

    println!("Cache directory: {}", cache_dir.display());

    // Count packages and calculate size
    let mut total_size: u64 = 0;
    let mut package_count = 0;

    if let Ok(entries) = fs::read_dir(&cache_dir) {
        for entry in entries.flatten() {
            if entry.path().is_dir() {
                package_count += 1;
                // Calculate directory size
                if let Ok(size) = dir_size(&entry.path()) {
                    total_size += size;
                }
            }
        }
    }

    println!("Total size: {}", format_size(total_size));
    println!("Packages: {}", package_count);

    Ok(())
}

/// List all cached packages
pub fn run_list() -> HorusResult<()> {
    let cache_dir = crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

    println!("{}", "Cached Packages".cyan().bold());
    println!("{}", "─".repeat(60));

    if !cache_dir.exists() {
        println!("  (cache is empty)");
        return Ok(());
    }

    let mut packages: Vec<_> = fs::read_dir(&cache_dir)
        .map_err(|e| HorusError::Config(e.to_string()))?
        .filter_map(|e| e.ok())
        .filter(|e| e.path().is_dir())
        .collect();

    packages.sort_by_key(|e| e.file_name());

    if packages.is_empty() {
        println!("  (cache is empty)");
    } else {
        for entry in packages {
            let name = entry.file_name().to_string_lossy().to_string();
            let size = dir_size(&entry.path()).unwrap_or(0);
            println!("  {} {}", name.yellow(), format_size(size).dimmed());
        }
    }

    Ok(())
}

/// Clean unused packages from cache
pub fn run_clean(dry_run: bool) -> HorusResult<()> {
    let cache_dir = crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

    println!("{} Scanning for unused packages...", "[CACHE]".cyan());

    if !cache_dir.exists() {
        println!("Cache is empty, nothing to clean.");
        return Ok(());
    }

    // Find all workspaces and their dependencies
    let registry =
        workspace::WorkspaceRegistry::load().map_err(|e| HorusError::Config(e.to_string()))?;

    let mut used_packages: std::collections::HashSet<String> = std::collections::HashSet::new();

    for ws in &registry.workspaces {
        let yaml_path = ws.path.join("horus.yaml");
        if yaml_path.exists() {
            if let Ok(deps) = crate::commands::run::parse_horus_yaml_dependencies_v2(
                yaml_path.to_str().unwrap_or(""),
            ) {
                for dep in deps {
                    // Extract package name from dependency spec
                    used_packages.insert(dep.name.clone());
                }
            }
        }
    }

    // Find cached packages not in use
    let mut to_remove = Vec::new();
    let mut freed_size: u64 = 0;

    if let Ok(entries) = fs::read_dir(&cache_dir) {
        for entry in entries.flatten() {
            if entry.path().is_dir() {
                let name = entry.file_name().to_string_lossy().to_string();
                let pkg_base = name.split('@').next().unwrap_or(&name);

                if !used_packages.contains(pkg_base) {
                    let size = dir_size(&entry.path()).unwrap_or(0);
                    freed_size += size;
                    to_remove.push((entry.path(), name, size));
                }
            }
        }
    }

    if to_remove.is_empty() {
        println!("{} All cached packages are in use.", "✓".green());
        return Ok(());
    }

    println!("\nUnused packages:");
    for (_, name, size) in &to_remove {
        println!("  {} {} ({})", "×".red(), name, format_size(*size));
    }
    println!("\nTotal to free: {}", format_size(freed_size).green());

    if dry_run {
        println!("\n{} Dry run - no files removed.", "[DRY]".yellow());
    } else {
        for (path, name, _) in &to_remove {
            if let Err(e) = fs::remove_dir_all(path) {
                println!("  {} Failed to remove {}: {}", "!".red(), name, e);
            }
        }
        println!(
            "\n{} Removed {} packages, freed {}.",
            "✓".green(),
            to_remove.len(),
            format_size(freed_size)
        );
    }

    Ok(())
}

/// Purge the entire cache
pub fn run_purge(yes: bool) -> HorusResult<()> {
    let cache_dir = crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

    if !cache_dir.exists() {
        println!("Cache is already empty.");
        return Ok(());
    }

    let total_size = dir_size(&cache_dir).unwrap_or(0);
    let count = fs::read_dir(&cache_dir).map(|e| e.count()).unwrap_or(0);

    println!(
        "{} This will remove ALL cached packages:",
        "[WARN]".yellow().bold()
    );
    println!("  Packages: {}", count);
    println!("  Size: {}", format_size(total_size));
    println!();

    if !yes {
        print!("Continue? [y/N] ");
        use std::io::Write;
        std::io::stdout().flush().ok();

        let mut input = String::new();
        std::io::stdin().read_line(&mut input).ok();

        if !input.trim().eq_ignore_ascii_case("y") {
            println!("Aborted.");
            return Ok(());
        }
    }

    fs::remove_dir_all(&cache_dir)
        .map_err(|e| HorusError::Config(format!("Failed to purge cache: {}", e)))?;

    println!(
        "{} Cache purged. Freed {}.",
        "✓".green(),
        format_size(total_size)
    );
    Ok(())
}
