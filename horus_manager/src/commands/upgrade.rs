//! `horus upgrade` — upgrade horus CLI and plugins.
//!
//! Checks for newer versions of horus and installed plugins,
//! downloads and installs updates.

use anyhow::Result;
use colored::*;

/// Run `horus upgrade`.
///
/// - `check_only`: If true, show available updates without installing.
pub fn run_upgrade(check_only: bool) -> Result<()> {
    let current_version = env!("CARGO_PKG_VERSION");

    println!("{}", "horus upgrade".bold());
    println!();
    println!("  Current version: {}", current_version.cyan());

    // ── Phase 1: Check for horus CLI updates ─────────────────────────────
    println!("  Checking for updates...");
    match check_latest_version() {
        Ok(Some(latest)) => {
            if latest != current_version {
                println!(
                    "  {} New version available: {} → {}",
                    "!".yellow(),
                    current_version.dimmed(),
                    latest.green()
                );
                if !check_only {
                    upgrade_horus(&latest)?;
                }
            } else {
                println!("  {} horus is up to date", "✓".green());
            }
        }
        Ok(None) => {
            println!("  {} Could not determine latest version", "!".yellow());
        }
        Err(e) => {
            println!("  {} Failed to check updates: {}", "!".yellow(), e);
        }
    }

    // ── Phase 2: Check plugin updates ────────────────────────────────────
    let plugins = list_installed_plugins();
    if !plugins.is_empty() {
        println!();
        println!("  {}", "Plugins:".bold());
        for plugin in &plugins {
            println!("    {} {}", "•".dimmed(), plugin);
        }
        if !check_only {
            println!("  Plugin updates not yet implemented.");
        }
    }

    Ok(())
}

/// Check the latest horus version from the registry API.
fn check_latest_version() -> Result<Option<String>> {
    // Try to fetch from registry
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(5))
        .build()?;

    let resp = client
        .get("https://horus-registry.dev/api/packages/horus/latest")
        .send();

    match resp {
        Ok(r) if r.status().is_success() => {
            let body: serde_json::Value = r.json()?;
            Ok(body["version"].as_str().map(String::from))
        }
        _ => Ok(None),
    }
}

/// Upgrade horus by rebuilding from source.
fn upgrade_horus(version: &str) -> Result<()> {
    println!("  Building horus {}...", version.cyan());

    // Try cargo install from source
    let status = std::process::Command::new("cargo")
        .args(["install", "--path", "."])
        .status();

    match status {
        Ok(s) if s.success() => {
            println!("  {} Upgraded to {}", "✓".green(), version.green());
        }
        _ => {
            println!(
                "  {} Auto-upgrade failed. Manual upgrade: {}",
                "!".yellow(),
                "cargo install --path .".dimmed()
            );
        }
    }

    Ok(())
}

/// List installed plugins from the global and local registries.
fn list_installed_plugins() -> Vec<String> {
    let mut plugins = Vec::new();

    // Check global plugins
    if let Some(home) = dirs::home_dir() {
        let global = home.join(".horus/plugins");
        if let Ok(entries) = std::fs::read_dir(global) {
            for entry in entries.flatten() {
                if let Some(name) = entry.file_name().to_str() {
                    plugins.push(format!("{} (global)", name));
                }
            }
        }
    }

    // Check local plugins
    let local = std::path::Path::new(".horus/plugins");
    if let Ok(entries) = std::fs::read_dir(local) {
        for entry in entries.flatten() {
            if let Some(name) = entry.file_name().to_str() {
                plugins.push(format!("{} (local)", name));
            }
        }
    }

    plugins
}
