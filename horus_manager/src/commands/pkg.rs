//! Package management commands including plugin support
//!
//! This module provides helper functions for pkg subcommands that
//! involve plugin management.

use crate::cargo_utils::is_executable;
use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::manifest::{HorusManifest, HORUS_TOML};
use crate::plugins::{
    CommandInfo, Compatibility, PluginEntry, PluginRegistry, PluginResolver, PluginScope,
    PluginSource, VerificationStatus, HORUS_VERSION,
};
use anyhow::{anyhow, Result};
use chrono::Utc;
use colored::*;
use serde_json;
use std::fs;
use std::io::IsTerminal;
use std::path::{Path, PathBuf};

/// Detect if a package has CLI plugin capabilities
///
/// Returns Some(PluginMetadata) if the package provides CLI extensions
pub fn detect_plugin_metadata(package_dir: &Path) -> Option<PluginMetadata> {
    // Check for horus.toml with plugin configuration
    let horus_toml_path = package_dir.join(HORUS_TOML);
    if horus_toml_path.exists() {
        match fs::read_to_string(&horus_toml_path) {
            Ok(content) => match content.parse::<toml::Table>() {
                Ok(toml_val) => {
                    if let Some(plugin) = toml_val.get("plugin") {
                        return parse_plugin_toml_manifest(plugin, &toml_val, package_dir);
                    }
                }
                Err(e) => {
                    log::warn!("Failed to parse {}: {}", horus_toml_path.display(), e);
                }
            },
            Err(e) => {
                log::warn!("Failed to read {}: {}", horus_toml_path.display(), e);
            }
        }
    }

    // Check for Cargo.toml with [package.metadata.horus] plugin config
    let cargo_toml = package_dir.join(CARGO_TOML);
    if cargo_toml.exists() {
        match fs::read_to_string(&cargo_toml) {
            Ok(content) => match content.parse::<toml::Table>() {
                Ok(toml) => {
                    if let Some(metadata) = toml
                        .get("package")
                        .and_then(|p| p.get("metadata"))
                        .and_then(|m| m.get("horus"))
                    {
                        return parse_plugin_toml(metadata, &toml, package_dir);
                    }
                }
                Err(e) => {
                    log::warn!("Failed to parse {}: {}", cargo_toml.display(), e);
                }
            },
            Err(e) => {
                log::warn!("Failed to read {}: {}", cargo_toml.display(), e);
            }
        }
    }

    // Check for bin directory with horus-* binaries
    let bin_dir = package_dir.join("bin");
    if bin_dir.exists() {
        if let Ok(entries) = fs::read_dir(&bin_dir) {
            for entry in entries.flatten() {
                if let Some(name) = entry.file_name().to_str() {
                    if let Some(command) = name.strip_prefix("horus-") {
                        if is_executable(&entry.path()) {
                            let command = command.to_string();
                            return Some(PluginMetadata {
                                command,
                                binary: entry.path(),
                                package_name: package_dir
                                    .file_name()
                                    .and_then(|n| n.to_str())
                                    .unwrap_or("unknown")
                                    .to_string(),
                                version: detect_version(package_dir)
                                    .unwrap_or_else(|| "0.0.0".to_string()),
                                commands: vec![],
                                compatibility: Compatibility::default(),
                                permissions: vec![],
                            });
                        }
                    }
                }
            }
        }
    }

    // Auto-detect from Cargo.toml: any horus-* package with a [[bin]] entry
    // is automatically a plugin. The command name is the binary name (or the
    // package name with "horus-" stripped). This means `horus-sim3d` with
    // `[[bin]] name = "sim3d"` becomes a plugin providing the `sim3d` command
    // with zero extra configuration from the package author.
    if cargo_toml.exists() {
        if let Ok(content) = fs::read_to_string(&cargo_toml) {
            if let Ok(toml) = content.parse::<toml::Table>() {
                let package_name = toml
                    .get("package")
                    .and_then(|p| p.get("name"))
                    .and_then(|n| n.as_str())
                    .unwrap_or("");

                if package_name.starts_with("horus-") {
                    // Find the first [[bin]] entry
                    let bin_name = toml.get("bin").and_then(|b| b.as_array()).and_then(|bins| {
                        bins.first()
                            .and_then(|b| b.get("name"))
                            .and_then(|n| n.as_str())
                    });

                    let command = bin_name.map(|s| s.to_string()).unwrap_or_else(|| {
                        package_name
                            .strip_prefix("horus-")
                            .unwrap_or(package_name)
                            .to_string()
                    });

                    // Look for the built binary
                    if let Some(binary) = find_binary(package_dir, &command) {
                        let version = toml
                            .get("package")
                            .and_then(|p| p.get("version"))
                            .and_then(|v| v.as_str())
                            .unwrap_or("0.0.0")
                            .to_string();

                        let description = toml
                            .get("package")
                            .and_then(|p| p.get("description"))
                            .and_then(|d| d.as_str())
                            .unwrap_or("")
                            .to_string();

                        return Some(PluginMetadata {
                            command: command.clone(),
                            binary,
                            package_name: package_name.to_string(),
                            version,
                            commands: vec![CommandInfo {
                                name: command,
                                description,
                            }],
                            compatibility: Compatibility::default(),
                            permissions: vec![],
                        });
                    }
                }
            }
        }
    }

    None
}

/// Plugin metadata extracted from package
#[derive(Debug, Clone)]
pub struct PluginMetadata {
    pub command: String,
    pub binary: PathBuf,
    pub package_name: String,
    pub version: String,
    pub commands: Vec<CommandInfo>,
    pub compatibility: Compatibility,
    pub permissions: Vec<String>,
}

fn parse_plugin_toml_manifest(
    plugin: &toml::Value,
    toml_table: &toml::Table,
    package_dir: &Path,
) -> Option<PluginMetadata> {
    let command = plugin.get("command")?.as_str()?.to_string();
    let binary_rel = plugin.get("binary")?.as_str()?;
    let binary = package_dir.join(binary_rel);

    let package_name = toml_table
        .get("package")
        .and_then(|p| p.get("name"))
        .and_then(|n| n.as_str())
        .unwrap_or("unknown")
        .to_string();

    let version = toml_table
        .get("package")
        .and_then(|p| p.get("version"))
        .and_then(|v| v.as_str())
        .unwrap_or("0.0.0")
        .to_string();

    let commands = plugin
        .get("subcommands")
        .and_then(|s| s.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|item| {
                    let name = item.get("name")?.as_str()?.to_string();
                    let description = item
                        .get("description")
                        .and_then(|d| d.as_str())
                        .unwrap_or("")
                        .to_string();
                    Some(CommandInfo { name, description })
                })
                .collect()
        })
        .unwrap_or_default();

    let compatibility = plugin
        .get("compatibility")
        .map(|c| Compatibility {
            horus_min: c
                .get("horus")
                .and_then(|h| h.as_str())
                .and_then(|s| s.split(',').next())
                .map(|s| s.trim_start_matches(">=").trim().to_string())
                .unwrap_or_else(|| "0.1.0".to_string()),
            horus_max: c
                .get("horus")
                .and_then(|h| h.as_str())
                .and_then(|s| s.split(',').nth(1))
                .map(|s| s.trim_start_matches('<').trim().to_string())
                .unwrap_or_else(|| "2.0.0".to_string()),
            platforms: c
                .get("platforms")
                .and_then(|p| p.as_array())
                .map(|arr| {
                    arr.iter()
                        .filter_map(|v| v.as_str().map(|s| s.to_string()))
                        .collect()
                })
                .unwrap_or_default(),
        })
        .unwrap_or_default();

    let permissions = plugin
        .get("permissions")
        .and_then(|p| p.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|v| v.as_str().map(|s| s.to_string()))
                .collect()
        })
        .unwrap_or_default();

    Some(PluginMetadata {
        command,
        binary,
        package_name,
        version,
        commands,
        compatibility,
        permissions,
    })
}

fn parse_plugin_toml(
    metadata: &toml::Value,
    toml: &toml::Table,
    package_dir: &Path,
) -> Option<PluginMetadata> {
    let cli_extension = metadata.get("cli_extension")?.as_bool()?;
    if !cli_extension {
        return None;
    }

    let command = metadata.get("command_name")?.as_str()?.to_string();
    let default_binary = format!("horus-{}", command);
    let binary_name = metadata
        .get("binary")
        .and_then(|b| b.as_str())
        .unwrap_or(&default_binary);

    // Look for binary in various locations
    let binary = find_binary(package_dir, binary_name)?;

    let package_name = toml
        .get("package")
        .and_then(|p| p.get("name"))
        .and_then(|n| n.as_str())
        .unwrap_or("unknown")
        .to_string();

    let version = toml
        .get("package")
        .and_then(|p| p.get("version"))
        .and_then(|v| v.as_str())
        .unwrap_or("0.0.0")
        .to_string();

    let commands = metadata
        .get("subcommands")
        .and_then(|s| s.as_array())
        .map(|arr| {
            arr.iter()
                .filter_map(|item| {
                    let name = item.get("name")?.as_str()?.to_string();
                    let description = item
                        .get("description")
                        .and_then(|d| d.as_str())
                        .unwrap_or("")
                        .to_string();
                    Some(CommandInfo { name, description })
                })
                .collect()
        })
        .unwrap_or_default();

    Some(PluginMetadata {
        command,
        binary,
        package_name,
        version,
        commands,
        compatibility: Compatibility::default(),
        permissions: vec![],
    })
}

fn find_binary(package_dir: &Path, binary_name: &str) -> Option<PathBuf> {
    // Check various locations
    let candidates = [
        package_dir.join("bin").join(binary_name),
        package_dir.join("target/release").join(binary_name),
        package_dir.join("target/debug").join(binary_name),
        package_dir.join(binary_name),
    ];

    candidates
        .into_iter()
        .find(|candidate| candidate.exists() && is_executable(candidate))
}

fn detect_version(package_dir: &Path) -> Option<String> {
    // Try horus.toml
    let horus_toml_path = package_dir.join(HORUS_TOML);
    if horus_toml_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(&horus_toml_path) {
            return Some(manifest.package.version);
        }
    }

    // Try Cargo.toml
    let cargo_toml = package_dir.join(CARGO_TOML);
    if cargo_toml.exists() {
        if let Ok(content) = fs::read_to_string(&cargo_toml) {
            if let Ok(toml) = content.parse::<toml::Table>() {
                if let Some(version) = toml
                    .get("package")
                    .and_then(|p| p.get("version"))
                    .and_then(|v| v.as_str())
                {
                    return Some(version.to_string());
                }
            }
        }
    }

    // Try metadata.json
    let metadata_json = package_dir.join("metadata.json");
    if metadata_json.exists() {
        if let Ok(content) = fs::read_to_string(&metadata_json) {
            if let Ok(json) = serde_json::from_str::<serde_json::Value>(&content) {
                if let Some(version) = json.get("version").and_then(|v| v.as_str()) {
                    return Some(version.to_string());
                }
            }
        }
    }

    None
}

/// Resolve the installed package directory after installation
pub fn resolve_installed_package_dir(name: &str, version: &str, global: bool) -> Option<PathBuf> {
    if global {
        let home = dirs::home_dir()?;
        let cache = home.join(".horus/cache");
        let versioned = cache.join(format!("{}@{}", name, version));
        if versioned.exists() {
            return Some(versioned);
        }
        let plain = cache.join(name);
        if plain.exists() {
            return Some(plain);
        }
    } else {
        let workspace_root = workspace::find_workspace_root().unwrap_or_else(|| PathBuf::from("."));
        let local = workspace_root.join(".horus/packages").join(name);
        if local.exists() {
            return Some(local);
        }
    }
    None
}

/// Register a plugin after package installation
pub fn register_plugin_after_install(
    package_dir: &Path,
    source: PluginSource,
    is_global: bool,
    project_root: Option<&Path>,
) -> Result<Option<String>> {
    let metadata = match detect_plugin_metadata(package_dir) {
        Some(m) => m,
        None => return Ok(None), // Not a plugin package
    };

    // Create plugin entry
    let checksum = PluginRegistry::calculate_checksum(&metadata.binary)?;

    // Clone commands for later display
    let commands_for_display = metadata.commands.clone();

    let entry = PluginEntry {
        package: metadata.package_name.clone(),
        version: metadata.version.clone(),
        source,
        binary: metadata.binary.clone(),
        checksum,
        signature: None,
        installed_at: Utc::now(),
        installed_by: HORUS_VERSION.to_string(),
        compatibility: metadata.compatibility,
        commands: metadata.commands,
        permissions: metadata.permissions,
    };

    // Update plugin registry FIRST (can be rolled back if symlink fails)
    let mut resolver = PluginResolver::new()?;

    if is_global {
        resolver
            .global_mut()
            .register_plugin(&metadata.command, entry);
        resolver.save_global()?;
    } else if let Some(root) = project_root {
        let project_name = root
            .file_name()
            .and_then(|n| n.to_str())
            .unwrap_or("unknown");
        let project_registry = resolver.get_or_create_project(project_name);
        project_registry.register_plugin(&metadata.command, entry);

        let path = PluginRegistry::project_path(root);
        project_registry.save_to(&path)?;
    }

    // Create symlink in bin directory (after registry is updated)
    let bin_dir = if is_global {
        PluginRegistry::global_bin_dir()?
    } else {
        project_root
            .map(PluginRegistry::project_bin_dir)
            .ok_or_else(|| anyhow!("No project root for local plugin"))?
    };

    fs::create_dir_all(&bin_dir)?;
    let symlink_path = bin_dir.join(format!("horus-{}", metadata.command));

    // Remove existing symlink
    if symlink_path.exists() || symlink_path.symlink_metadata().is_ok() {
        fs::remove_file(&symlink_path).ok();
    }

    // Create new symlink; roll back registry entry on failure
    if let Err(e) = horus_sys::fs::symlink(&metadata.binary, &symlink_path) {
        // Roll back registry entry
        log::warn!(
            "Symlink creation failed, rolling back plugin registration: {}",
            e
        );
        let mut rollback_resolver = PluginResolver::new().ok();
        if is_global {
            if let Some(ref mut r) = rollback_resolver {
                r.global_mut().unregister_plugin(&metadata.command);
                let _ = r.save_global();
            }
        }
        return Err(anyhow!("Failed to create plugin symlink: {}", e));
    }

    println!(
        "   {} Registered CLI plugin: {}",
        cli_output::ICON_INFO.cyan(),
        format!("horus {}", metadata.command).green()
    );

    if !commands_for_display.is_empty() {
        println!("      Commands:");
        for cmd in &commands_for_display {
            println!("        • {} - {}", cmd.name, cmd.description.dimmed());
        }
    }

    Ok(Some(metadata.command))
}

/// Unregister a plugin when package is removed
pub fn unregister_plugin(
    command: &str,
    is_global: bool,
    project_root: Option<&Path>,
) -> Result<()> {
    // Remove symlink FIRST (harmless orphan registry entry if this fails)
    let bin_dir = if is_global {
        PluginRegistry::global_bin_dir()?
    } else {
        project_root
            .map(PluginRegistry::project_bin_dir)
            .ok_or_else(|| anyhow!("No project root"))?
    };

    let symlink_path = bin_dir.join(format!("horus-{}", command));
    if symlink_path.exists() || symlink_path.symlink_metadata().is_ok() {
        fs::remove_file(&symlink_path)?;
    }

    // Then remove from registry (orphan symlinks are harmless, orphan registry entries aren't)
    let mut resolver = PluginResolver::new()?;

    if is_global {
        if resolver.global_mut().unregister_plugin(command).is_some() {
            resolver.save_global()?;
        }
    } else if let Some(root) = project_root {
        if let Some(project) = resolver.project_mut() {
            if project.unregister_plugin(command).is_some() {
                let path = PluginRegistry::project_path(root);
                project.save_to(&path)?;
            }
        }
    }

    println!(
        "   {} Unregistered plugin: {}",
        cli_output::ICON_HINT.dimmed(),
        command.yellow()
    );

    Ok(())
}

/// Enable a disabled plugin
pub fn enable_plugin(command: &str) -> Result<()> {
    let mut resolver = PluginResolver::new()?;

    // Get project root before mutable borrow
    let project_root = resolver.project_root().map(|p| p.to_path_buf());

    // Try project first, then global
    if let Some(project) = resolver.project_mut() {
        if project.is_disabled(command) {
            project.enable_plugin(command)?;
            if let Some(root) = &project_root {
                let path = PluginRegistry::project_path(root);
                project.save_to(&path)?;
            }
            println!(
                "{} Plugin '{}' enabled",
                cli_output::ICON_SUCCESS.green(),
                command.green()
            );
            return Ok(());
        }
    }

    if resolver.global().is_disabled(command) {
        resolver.global_mut().enable_plugin(command)?;
        resolver.save_global()?;
        println!(
            "{} Plugin '{}' enabled",
            cli_output::ICON_SUCCESS.green(),
            command.green()
        );
        return Ok(());
    }

    // Distinguish "not found" from "already enabled"
    let exists_in_project = resolver
        .project()
        .map(|p| p.get_plugin(command).is_some())
        .unwrap_or(false);
    let exists_in_global = resolver.global().get_plugin(command).is_some();

    if exists_in_project || exists_in_global {
        Err(anyhow!("Plugin '{}' is already enabled", command))
    } else {
        Err(anyhow!("Plugin '{}' not found", command))
    }
}

/// Disable a plugin without uninstalling
pub fn disable_plugin(command: &str, reason: Option<&str>) -> Result<()> {
    let mut resolver = PluginResolver::new()?;
    let reason = reason.unwrap_or("Disabled by user");

    // Get project root before mutable borrow
    let project_root = resolver.project_root().map(|p| p.to_path_buf());

    // Try project first, then global
    if let Some(project) = resolver.project_mut() {
        if project.get_plugin(command).is_some() {
            project.disable_plugin(command, reason)?;
            if let Some(root) = &project_root {
                let path = PluginRegistry::project_path(root);
                project.save_to(&path)?;
            }
            println!(
                "{} Plugin '{}' disabled (package still installed)",
                cli_output::ICON_WARN.yellow(),
                command.yellow()
            );
            return Ok(());
        }
    }

    if resolver.global().get_plugin(command).is_some() {
        resolver.global_mut().disable_plugin(command, reason)?;
        resolver.save_global()?;
        println!(
            "{} Plugin '{}' disabled (package still installed)",
            cli_output::ICON_WARN.yellow(),
            command.yellow()
        );
        return Ok(());
    }

    Err(anyhow!("Plugin '{}' not found", command))
}

/// Verify plugin integrity
pub fn verify_plugins(plugin_name: Option<&str>, json: bool) -> Result<()> {
    let resolver = PluginResolver::new()?;
    let results = resolver.verify_all();

    if json {
        let plugin_results: Vec<_> = results
            .iter()
            .filter(|r| plugin_name.map(|n| r.command == n).unwrap_or(true))
            .map(|r| {
                let status_str = match r.status {
                    VerificationStatus::Valid => "valid",
                    VerificationStatus::ChecksumMismatch => "checksum_mismatch",
                    VerificationStatus::Error => "error",
                };
                let scope_str = match r.scope {
                    PluginScope::Global => "global",
                    PluginScope::Project => "project",
                };
                serde_json::json!({
                    "command": r.command,
                    "scope": scope_str,
                    "status": status_str,
                    "error": r.error,
                })
            })
            .collect();
        let all_valid = plugin_results.iter().all(|r| r["status"] == "valid");
        let output = serde_json::json!({
            "plugins": plugin_results,
            "all_valid": all_valid,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return if all_valid {
            Ok(())
        } else {
            Err(anyhow!("Some plugins failed verification."))
        };
    }

    if results.is_empty() {
        println!("{} No plugins installed", cli_output::ICON_INFO.cyan());
        return Ok(());
    }

    println!("{} Verifying plugins...\n", cli_output::ICON_INFO.cyan());

    let mut all_valid = true;

    for result in &results {
        // Filter by name if specified
        if let Some(name) = plugin_name {
            if result.command != name {
                continue;
            }
        }

        let scope_str = match result.scope {
            PluginScope::Global => "(global)".dimmed(),
            PluginScope::Project => "(project)".dimmed(),
        };

        match result.status {
            VerificationStatus::Valid => {
                println!(
                    "  {} {} {} checksum OK",
                    cli_output::ICON_SUCCESS.green(),
                    result.command.green(),
                    scope_str
                );
            }
            VerificationStatus::ChecksumMismatch => {
                all_valid = false;
                println!(
                    "  {} {} {} checksum MISMATCH",
                    cli_output::ICON_ERROR.red(),
                    result.command.red(),
                    scope_str
                );
            }
            VerificationStatus::Error => {
                all_valid = false;
                println!(
                    "  {} {} {} error: {}",
                    cli_output::ICON_ERROR.red(),
                    result.command.red(),
                    scope_str,
                    result.error.as_deref().unwrap_or("unknown error")
                );
            }
        }
    }

    println!();

    if all_valid {
        println!(
            "{} All plugins verified successfully",
            cli_output::ICON_SUCCESS.green().bold()
        );
        Ok(())
    } else {
        Err(anyhow!(
            "Some plugins failed verification. Run 'horus install <package>' to reinstall."
        ))
    }
}

// ── Package management command handlers ──────────────────────────────────

use crate::{registry, workspace};
use horus_core::error::{ConfigError, HorusError, HorusResult};

/// Read the package name from a directory by checking horus.toml, then Cargo.toml
fn read_package_name_from_path(path: &Path) -> Result<String> {
    // Try horus.toml first
    let horus_toml_path = path.join(HORUS_TOML);
    if horus_toml_path.exists() {
        if let Ok(manifest) = HorusManifest::load_from(&horus_toml_path) {
            return Ok(manifest.package.name);
        }
    }

    // Try Cargo.toml
    let cargo_toml_path = path.join(CARGO_TOML);
    if cargo_toml_path.exists() {
        let content = fs::read_to_string(&cargo_toml_path)?;
        if let Ok(toml_val) = content.parse::<toml::Table>() {
            if let Some(name) = toml_val
                .get("package")
                .and_then(|p| p.get("name"))
                .and_then(|n| n.as_str())
            {
                return Ok(name.to_string());
            }
        }
    }

    // Fall back to directory name
    path.file_name()
        .and_then(|n| n.to_str())
        .map(|s| s.to_string())
        .ok_or_else(|| {
            anyhow!(
                "Cannot determine package name from path: {}",
                path.display()
            )
        })
}

/// Install a package from registry or local path
pub fn run_install(
    package: String,
    ver: Option<String>,
    global: bool,
    target: Option<String>,
    source: Option<String>,
    features: Option<Vec<String>>,
    dev: bool,
) -> HorusResult<()> {
    use crate::manifest::{DepSource, DependencyValue, DetailedDependency};

    // Auto-detect source from package string or --source flag
    let is_path = package.contains('/') || package.starts_with('.') || package.starts_with('~');
    let is_git = package.starts_with("https://") || package.starts_with("git://");

    let dep_source = if let Some(ref s) = source {
        match s.to_lowercase().as_str() {
            "crates.io" | "crates" | "cargo" => DepSource::CratesIo,
            "pypi" | "pip" | "python" => DepSource::PyPI,
            "path" => DepSource::Path,
            "git" => DepSource::Git,
            "system" | "apt" | "brew" => DepSource::System,
            "registry" | "horus" => DepSource::Registry,
            _ => {
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Unknown source '{}'. Use: crates.io, pypi, path, git, system, registry",
                    s
                ))));
            }
        }
    } else if is_path {
        DepSource::Path
    } else if is_git {
        DepSource::Git
    } else {
        // Smart resolution: detect whether it's a crates.io, PyPI, or registry package
        use crate::source_resolver::PackageSourceResolver;
        let ctx = crate::dispatch::detect_context(&std::env::current_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?);
        let resolver = PackageSourceResolver::new(&ctx.languages);
        let resolved = resolver.resolve(&package);
        resolved.source
    };

    // For global installs, skip horus.toml and install directly
    if global {
        return run_install_global(&package, ver.as_deref(), target);
    }

    // ── Write to horus.toml ──────────────────────────────────────────────
    let manifest_path = Path::new(HORUS_TOML);
    let mut manifest = if manifest_path.exists() {
        HorusManifest::load_from(manifest_path)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
    } else {
        return Err(HorusError::Config(ConfigError::Other(
            "No horus.toml found. Run `horus new` to create a project first.".to_string(),
        )));
    };

    // Build the DependencyValue
    let dep_name: String;
    let dep_value: DependencyValue;

    match dep_source {
        DepSource::Path => {
            // Resolve and validate path
            let path = PathBuf::from(&package);
            let absolute_path = if path.is_absolute() {
                path.clone()
            } else {
                std::env::current_dir()
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                    .join(&path)
            };
            if !absolute_path.exists() {
                return Err(HorusError::Config(ConfigError::Other(format!(
                    "Path does not exist: {}",
                    absolute_path.display()
                ))));
            }

            dep_name = read_package_name_from_path(&absolute_path)
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

            dep_value = DependencyValue::Detailed(DetailedDependency {
                version: None,
                source: Some(DepSource::Path),
                features: features.unwrap_or_default(),
                optional: false,
                path: Some(package.clone()),
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: None,
                cmake_package: None,
                lang: None,
            });
        }
        DepSource::Git => {
            dep_name = package
                .rsplit('/')
                .next()
                .unwrap_or(&package)
                .trim_end_matches(".git")
                .to_string();

            dep_value = DependencyValue::Detailed(DetailedDependency {
                version: None,
                source: Some(DepSource::Git),
                features: features.unwrap_or_default(),
                optional: false,
                path: None,
                git: Some(package.clone()),
                branch: None,
                tag: None,
                rev: None,
                apt: None,
                cmake_package: None,
                lang: None,
            });
        }
        DepSource::CratesIo | DepSource::PyPI | DepSource::System => {
            dep_name = package.clone();
            let feats = features.unwrap_or_default();
            if feats.is_empty() && ver.is_none() {
                // Simple form: serde = { version = "*", source = "crates.io" }
                dep_value = DependencyValue::Detailed(DetailedDependency {
                    version: Some("*".to_string()),
                    source: Some(dep_source.clone()),
                    features: vec![],
                    optional: false,
                    path: None,
                    git: None,
                    branch: None,
                    tag: None,
                    rev: None,
                    apt: None,
                    cmake_package: None,
                    lang: None,
                });
            } else {
                dep_value = DependencyValue::Detailed(DetailedDependency {
                    version: ver.clone().or_else(|| Some("*".to_string())),
                    source: Some(dep_source.clone()),
                    features: feats,
                    optional: false,
                    path: None,
                    git: None,
                    branch: None,
                    tag: None,
                    rev: None,
                    apt: None,
                    cmake_package: None,
                    lang: None,
                });
            }
        }
        DepSource::Registry => {
            dep_name = package.clone();
            let feats = features.unwrap_or_default();
            if feats.is_empty() {
                // Simple form: rplidar = "1.2.0"
                dep_value = DependencyValue::Simple(
                    ver.clone().unwrap_or_else(|| "*".to_string()),
                );
            } else {
                dep_value = DependencyValue::Detailed(DetailedDependency {
                    version: ver.clone().or_else(|| Some("*".to_string())),
                    source: None, // Registry is the default
                    features: feats,
                    optional: false,
                    path: None,
                    git: None,
                    branch: None,
                    apt: None,
                    cmake_package: None,
                    lang: None,
                    tag: None,
                    rev: None,
                });
            }
        }
    }

    // Insert into the appropriate section
    let section_name = if dev { "dev-dependencies" } else { "dependencies" };
    let deps_map = if dev {
        &mut manifest.dev_dependencies
    } else {
        &mut manifest.dependencies
    };

    let was_present = deps_map.contains_key(&dep_name);
    deps_map.insert(dep_name.clone(), dep_value);

    // Save manifest
    manifest
        .save_to(manifest_path)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    let action = if was_present { "Updated" } else { "Added" };
    let version_str = ver
        .as_deref()
        .map(|v| format!("@{}", v))
        .unwrap_or_default();

    println!(
        "{} {} {} ({}) to [{}]",
        cli_output::ICON_SUCCESS.green(),
        action,
        format!("{}{}", dep_name, version_str).green(),
        dep_source,
        section_name,
    );

    // ── Physical installation for registry packages ──────────────────────
    if dep_source == DepSource::Registry {
        let install_target = if let Some(target_name) = target {
            let registry = workspace::WorkspaceRegistry::load()
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            let ws = registry.find_by_name(&target_name).ok_or_else(|| {
                HorusError::Config(ConfigError::Other(format!(
                    "Workspace '{}' not found",
                    target_name
                )))
            })?;
            workspace::InstallTarget::Local(ws.path.clone())
        } else {
            workspace::detect_or_select_workspace(true)
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
        };

        let client = registry::RegistryClient::new();
        let installed_version = client
            .install_to_target(&package, ver.as_deref(), install_target.clone())
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        // Auto-detect and register plugins
        let is_global = matches!(install_target, workspace::InstallTarget::Global);
        let project_root = match &install_target {
            workspace::InstallTarget::Local(p) => Some(p.clone()),
            workspace::InstallTarget::Global => None,
        };
        if let Some(pkg_dir) =
            resolve_installed_package_dir(&package, &installed_version, is_global)
        {
            match register_plugin_after_install(
                &pkg_dir,
                PluginSource::Registry,
                is_global,
                project_root.as_deref(),
            ) {
                Ok(Some(cmd)) => {
                    println!(
                        "  {} Registered plugin command: {}",
                        cli_output::ICON_SUCCESS.green(),
                        format!("horus {}", cmd).green()
                    );
                }
                Ok(None) => {}
                Err(e) => {
                    println!(
                        "  {} Plugin registration failed: {}",
                        cli_output::ICON_WARN.yellow(),
                        e
                    );
                }
            }
        }
    } else if dep_source == DepSource::CratesIo {
        println!(
            "  {} Cargo will fetch this crate on next build",
            cli_output::ICON_HINT.dimmed()
        );
    } else if dep_source == DepSource::PyPI {
        println!(
            "  {} pip will install this package on next build",
            cli_output::ICON_HINT.dimmed()
        );
    } else if dep_source == DepSource::System {
        // Attempt to install via apt (Debian/Ubuntu)
        println!(
            "  {} Installing system package via apt...",
            cli_output::ICON_HINT.dimmed()
        );
        let status = std::process::Command::new("sudo")
            .args(["apt", "install", "-y", &package])
            .status();
        match status {
            Ok(s) if s.success() => {
                println!(
                    "  {} Installed system package: {}",
                    cli_output::ICON_SUCCESS.green(),
                    package.green()
                );
            }
            Ok(s) => {
                eprintln!(
                    "  {} apt install failed (exit {}). Install manually: sudo apt install {}",
                    cli_output::ICON_WARN.yellow(),
                    s.code().unwrap_or(-1),
                    package
                );
            }
            Err(e) => {
                eprintln!(
                    "  {} Could not run apt: {}. Install manually: sudo apt install {}",
                    cli_output::ICON_WARN.yellow(),
                    e,
                    package
                );
            }
        }
    }

    Ok(())
}

/// Install a package globally (no horus.toml involved).
fn run_install_global(
    package: &str,
    ver: Option<&str>,
    target: Option<String>,
) -> HorusResult<()> {
    let install_target = if let Some(target_name) = target {
        let registry = workspace::WorkspaceRegistry::load()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
        let ws = registry.find_by_name(&target_name).ok_or_else(|| {
            HorusError::Config(ConfigError::Other(format!(
                "Workspace '{}' not found",
                target_name
            )))
        })?;
        workspace::InstallTarget::Local(ws.path.clone())
    } else {
        workspace::InstallTarget::Global
    };

    let client = registry::RegistryClient::new();
    let installed_version = client
        .install_to_target(package, ver, install_target.clone())
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    // Auto-detect and register plugins
    let is_global = matches!(install_target, workspace::InstallTarget::Global);
    let project_root = match &install_target {
        workspace::InstallTarget::Local(p) => Some(p.clone()),
        workspace::InstallTarget::Global => None,
    };
    if let Some(pkg_dir) =
        resolve_installed_package_dir(package, &installed_version, is_global)
    {
        match register_plugin_after_install(
            &pkg_dir,
            PluginSource::Registry,
            is_global,
            project_root.as_deref(),
        ) {
            Ok(Some(cmd)) => {
                println!(
                    "  {} Registered plugin command: {}",
                    cli_output::ICON_SUCCESS.green(),
                    format!("horus {}", cmd).green()
                );
            }
            Ok(None) => {}
            Err(e) => {
                println!(
                    "  {} Plugin registration failed: {}",
                    cli_output::ICON_WARN.yellow(),
                    e
                );
            }
        }
    }

    Ok(())
}

/// Remove a package
pub fn run_remove(package: String, global: bool, target: Option<String>) -> HorusResult<()> {
    // Confirm before removing (skip if stdin is not a terminal, e.g. scripts)
    if std::io::stdin().is_terminal() {
        use std::io::Write;
        let scope = if global { "globally" } else { "from workspace" };
        print!(
            "{} Remove {} {}? [y/N] ",
            cli_output::ICON_WARN.yellow(),
            package.yellow(),
            scope
        );
        std::io::stdout().flush().ok();
        let mut answer = String::new();
        std::io::stdin().read_line(&mut answer).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!("Failed to read input: {}", e)))
        })?;
        if !matches!(answer.trim().to_lowercase().as_str(), "y" | "yes") {
            println!("  Cancelled.");
            return Ok(());
        }
    }

    println!(
        "{} Removing {}...",
        cli_output::ICON_INFO.cyan(),
        package.yellow()
    );

    let remove_dir = if global {
        // Remove from global cache
        let global_cache = crate::paths::cache_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        // Find versioned directory
        let mut found = None;
        if global_cache.exists() {
            for entry in fs::read_dir(&global_cache)
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
            {
                let entry =
                    entry.map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
                let name = entry.file_name().to_string_lossy().to_string();
                if name == package || name.starts_with(&format!("{}@", package)) {
                    found = Some(entry.path());
                    break;
                }
            }
        }
        found.ok_or_else(|| {
            HorusError::Config(ConfigError::Other(format!(
                "Package {} not found in global cache",
                package
            )))
        })?
    } else if let Some(target_name) = &target {
        // Remove from specific workspace
        let reg = workspace::WorkspaceRegistry::load()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
        let ws = reg.find_by_name(target_name).ok_or_else(|| {
            HorusError::Config(ConfigError::Other(format!(
                "Workspace '{}' not found",
                target_name
            )))
        })?;
        ws.path.join(".horus/packages").join(&package)
    } else {
        // Remove from current workspace
        if let Some(root) = workspace::find_workspace_root() {
            root.join(".horus/packages").join(&package)
        } else {
            PathBuf::from(".horus/packages").join(&package)
        }
    };

    // Check for system package reference first
    let packages_dir = if global {
        crate::paths::cache_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
    } else if let Some(target_name) = &target {
        let reg = workspace::WorkspaceRegistry::load()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
        let ws = reg.find_by_name(target_name).ok_or_else(|| {
            HorusError::Config(ConfigError::Other(format!(
                "Workspace '{}' not found",
                target_name
            )))
        })?;
        ws.path.join(".horus/packages")
    } else if let Some(root) = workspace::find_workspace_root() {
        root.join(".horus/packages")
    } else {
        PathBuf::from(".horus/packages")
    };

    let system_ref = packages_dir.join(format!("{}.system.json", package));
    if system_ref.exists() {
        // Read to determine package type
        let content = fs::read_to_string(&system_ref).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to read system reference: {}",
                e
            )))
        })?;
        let metadata: serde_json::Value = serde_json::from_str(&content).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "failed to parse system reference: {}",
                e
            )))
        })?;

        // Remove reference file
        fs::remove_file(&system_ref).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to remove system reference: {}",
                e
            )))
        })?;

        // If it's a cargo package, also remove bin symlink
        if let Some(pkg_type) = metadata.get("package_type") {
            if pkg_type == "CratesIO" {
                let bin_dir = if let Some(root) = workspace::find_workspace_root() {
                    root.join(".horus/bin")
                } else {
                    PathBuf::from(".horus/bin")
                };
                let bin_link = bin_dir.join(&package);
                if bin_link.exists() || bin_link.read_link().is_ok() {
                    fs::remove_file(&bin_link).map_err(|e| {
                        HorusError::Config(ConfigError::Other(format!(
                            "Failed to remove binary link: {}",
                            e
                        )))
                    })?;
                    println!("  Removed binary link for {}", package);
                }
            }
        }

        println!("  Removed system package reference for {}", package);

        // Remove from horus.toml and horus.lock
        remove_from_horus_toml(&package);
        remove_from_horus_lock(&package);

        return Ok(());
    }

    // Check for crates.io tracking reference
    let cratesio_ref = packages_dir.join(format!("{}.crates-io.json", package));
    if cratesio_ref.exists() {
        let content = fs::read_to_string(&cratesio_ref).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to read crates.io reference: {}",
                e
            )))
        })?;
        let metadata: serde_json::Value = serde_json::from_str(&content).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to parse crates.io reference: {}",
                e
            )))
        })?;

        let install_type = metadata
            .get("install_type")
            .and_then(|v| v.as_str())
            .unwrap_or("bin");

        if install_type == "lib" {
            // Remove from horus.toml (the source of truth for deps)
            remove_from_horus_toml(&package);
        }

        // Delete tracking json
        fs::remove_file(&cratesio_ref).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to remove tracking file: {}",
                e
            )))
        })?;

        // Also remove the package directory if it exists (for binary installs)
        if remove_dir.exists() {
            fs::remove_dir_all(&remove_dir).ok();
        }

        remove_from_horus_lock(&package);

        println!(
            "{} Removed {} (crates.io)",
            cli_output::ICON_SUCCESS.green(),
            package
        );
        return Ok(());
    }

    // Check for PyPI tracking reference
    let pypi_ref = packages_dir.join(format!("{}.pypi.json", package));
    if pypi_ref.exists() {
        // Remove from horus.toml (the source of truth for deps)
        remove_from_horus_toml(&package);

        // Delete tracking json
        fs::remove_file(&pypi_ref).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to remove tracking file: {}",
                e
            )))
        })?;

        // Remove package directory
        if remove_dir.exists() {
            fs::remove_dir_all(&remove_dir).ok();
        }

        remove_from_horus_lock(&package);

        println!(
            "{} Removed {} (PyPI)",
            cli_output::ICON_SUCCESS.green(),
            package
        );
        return Ok(());
    }

    if !remove_dir.exists() {
        println!("  Package {} is not installed", package);
        return Ok(());
    }

    // Remove package directory
    fs::remove_dir_all(&remove_dir).map_err(|e| {
        HorusError::Config(ConfigError::Other(format!(
            "Failed to remove package: {}",
            e
        )))
    })?;

    // Remove from horus.toml and horus.lock
    remove_from_horus_toml(&package);
    remove_from_horus_lock(&package);

    println!("  Removed {} from {}", package, remove_dir.display());

    Ok(())
}

/// Remove a dependency from horus.toml if the manifest exists.
fn remove_from_horus_toml(package: &str) {
    use crate::manifest::{HorusManifest, HORUS_TOML};

    let manifest_path = std::path::Path::new(HORUS_TOML);
    if !manifest_path.exists() {
        return;
    }

    let Ok(mut manifest) = HorusManifest::load_from(manifest_path) else {
        return;
    };

    let had_dep = manifest.dependencies.remove(package).is_some();
    let had_dev = manifest.dev_dependencies.remove(package).is_some();

    if had_dep || had_dev {
        if manifest.save_to(manifest_path).is_ok() {
            println!(
                "  {} Removed {} from horus.toml",
                cli_output::ICON_SUCCESS.green(),
                package
            );
        }
    }
}

/// Remove a package from horus.lock if the lockfile exists.
fn remove_from_horus_lock(package: &str) {
    use crate::lockfile::{HorusLockfile, HORUS_LOCK};

    let lock_path = std::path::Path::new(HORUS_LOCK);
    if !lock_path.exists() {
        return;
    }

    let Ok(mut lockfile) = HorusLockfile::load_from(lock_path) else {
        return;
    };

    // Remove from all sources
    let before = lockfile.packages.len();
    lockfile.packages.retain(|p| p.name != package);

    if lockfile.packages.len() < before {
        lockfile.save_to(lock_path).ok();
    }
}

/// List packages (local, global, or search)
pub fn run_list(query: Option<String>, global: bool, all: bool, json: bool) -> HorusResult<()> {
    if json {
        return run_list_json(query, global, all);
    }
    let client = registry::RegistryClient::new();

    if let Some(q) = query {
        // Search registry marketplace
        println!(
            "{} Searching registry marketplace for '{}'...",
            cli_output::ICON_INFO.cyan(),
            q
        );
        let results = client
            .search(&q, None, None)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        if results.is_empty() {
            println!("  No packages found in marketplace matching '{}'", q);
        } else {
            println!(
                "\n{} Found {} package(s) in marketplace:\n",
                cli_output::ICON_SUCCESS.green(),
                results.len()
            );
            for pkg in results {
                println!(
                    "  {} {} - {}",
                    pkg.name.yellow().bold(),
                    pkg.version.dimmed(),
                    pkg.description.unwrap_or_default()
                );
            }
        }
    } else if all {
        // List both local and global packages
        let global_cache = crate::paths::cache_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        // Show local packages
        println!("{} Local packages:\n", cli_output::ICON_INFO.cyan());
        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
            root.join(".horus/packages")
        } else {
            PathBuf::from(".horus/packages")
        };

        if packages_dir.exists() {
            let mut has_local = false;
            for entry in fs::read_dir(&packages_dir)
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
            {
                let entry =
                    entry.map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
                let entry_path = entry.path();

                // Skip if it's a metadata file
                if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                    continue;
                }

                if entry
                    .file_type()
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                    .is_dir()
                    || entry
                        .file_type()
                        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                        .is_symlink()
                {
                    has_local = true;
                    let name = entry.file_name().to_string_lossy().to_string();
                    if !print_package_info(&packages_dir, &name, &entry_path) {
                        println!("  {}", name.yellow());
                    }
                }
            }
            if !has_local {
                println!("  No local packages");
            }
        } else {
            println!("  No local packages");
        }

        // Show global packages
        println!(
            "\n{} Global cache packages:\n",
            cli_output::ICON_INFO.cyan()
        );
        if global_cache.exists() {
            let mut has_global = false;
            for entry in fs::read_dir(&global_cache)
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
            {
                let entry =
                    entry.map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
                if entry
                    .file_type()
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                    .is_dir()
                {
                    has_global = true;
                    let name = entry.file_name().to_string_lossy().to_string();
                    println!("  {}", name.yellow());
                }
            }
            if !has_global {
                println!("  No global packages");
            }
        } else {
            println!("  No global packages");
        }
    } else if global {
        // List global cache packages
        println!("{} Global cache packages:\n", cli_output::ICON_INFO.cyan());
        let global_cache = crate::paths::cache_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        if !global_cache.exists() {
            println!("  No global packages yet");
            return Ok(());
        }

        for entry in fs::read_dir(&global_cache)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
        {
            let entry = entry.map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            if entry
                .file_type()
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                .is_dir()
            {
                let name = entry.file_name().to_string_lossy().to_string();
                println!("  {}", name.yellow());
            }
        }
    } else {
        // List local workspace packages (default)
        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
            root.join(".horus/packages")
        } else {
            PathBuf::from(".horus/packages")
        };

        println!("{} Local packages:\n", cli_output::ICON_INFO.cyan());

        if !packages_dir.exists() {
            println!("  No packages installed yet");
            return Ok(());
        }

        for entry in fs::read_dir(&packages_dir)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
        {
            let entry = entry.map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            let entry_path = entry.path();

            // Skip if it's a metadata file
            if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                continue;
            }

            if entry
                .file_type()
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                .is_dir()
                || entry
                    .file_type()
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
                    .is_symlink()
            {
                let name = entry.file_name().to_string_lossy().to_string();
                if !print_package_info(&packages_dir, &name, &entry_path) {
                    println!("  {}", name.yellow());
                }
            }
        }
    }

    Ok(())
}

/// JSON output for the list command
fn run_list_json(_query: Option<String>, global: bool, all: bool) -> HorusResult<()> {
    let mut local_packages = Vec::new();
    let mut global_packages = Vec::new();

    // Collect local packages
    if !global || all {
        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
            root.join(".horus/packages")
        } else {
            PathBuf::from(".horus/packages")
        };
        if packages_dir.exists() {
            if let Ok(entries) = fs::read_dir(&packages_dir) {
                for entry in entries.flatten() {
                    let entry_path = entry.path();
                    if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                        continue;
                    }
                    if entry_path.is_dir() || entry_path.is_symlink() {
                        let name = entry.file_name().to_string_lossy().to_string();
                        local_packages.push(serde_json::json!({
                            "name": name,
                            "scope": "local",
                        }));
                    }
                }
            }
        }
    }

    // Collect global packages
    if global || all {
        let global_cache = crate::paths::cache_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
        if global_cache.exists() {
            if let Ok(entries) = fs::read_dir(&global_cache) {
                for entry in entries.flatten() {
                    if entry.path().is_dir() {
                        let name = entry.file_name().to_string_lossy().to_string();
                        global_packages.push(serde_json::json!({
                            "name": name,
                            "scope": "global",
                        }));
                    }
                }
            }
        }
    }

    let mut packages = local_packages;
    packages.extend(global_packages);

    let output = serde_json::json!({ "packages": packages });
    println!(
        "{}",
        serde_json::to_string_pretty(&output).unwrap_or_default()
    );
    Ok(())
}

/// Print package info from metadata files, returns true if info was printed
fn print_package_info(packages_dir: &Path, name: &str, entry_path: &Path) -> bool {
    // Check for path dependency metadata
    let path_meta = packages_dir.join(format!("{}.path.json", name));
    if path_meta.exists() {
        if let Ok(content) = fs::read_to_string(&path_meta) {
            if let Ok(metadata) = serde_json::from_str::<serde_json::Value>(&content) {
                let version = metadata["version"].as_str().unwrap_or("dev");
                let path = metadata["source_path"].as_str().unwrap_or("unknown");
                println!(
                    "   {} {} {} {}",
                    name.yellow(),
                    version.dimmed(),
                    "(path:".dimmed(),
                    format!("{})", path).dimmed()
                );
                return true;
            }
        }
    }

    // Check for system package metadata
    let system_meta = packages_dir.join(format!("{}.system.json", name));
    if system_meta.exists() {
        if let Ok(content) = fs::read_to_string(&system_meta) {
            if let Ok(metadata) = serde_json::from_str::<serde_json::Value>(&content) {
                let version = metadata["version"].as_str().unwrap_or("unknown");
                println!(
                    "   {} {} {}",
                    name.yellow(),
                    version.dimmed(),
                    "(system)".dimmed()
                );
                return true;
            }
        }
    }

    // Check for regular metadata.json
    let metadata_path = entry_path.join("metadata.json");
    if metadata_path.exists() {
        if let Ok(content) = fs::read_to_string(&metadata_path) {
            if let Ok(metadata) = serde_json::from_str::<serde_json::Value>(&content) {
                let version = metadata["version"].as_str().unwrap_or("unknown");
                println!(
                    "   {} {} {}",
                    name.yellow(),
                    version.dimmed(),
                    "(registry)".dimmed()
                );
                return true;
            }
        }
    }

    false
}

/// Publish a package to the registry
pub fn run_publish(dry_run: bool) -> HorusResult<()> {
    let client = registry::RegistryClient::new();
    client
        .publish(None, dry_run, None)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    Ok(())
}

/// Update packages
pub fn run_update(package: Option<String>, global: bool, dry_run: bool) -> HorusResult<()> {
    let client = registry::RegistryClient::new();
    client
        .update_packages(package.as_deref(), global, dry_run)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
    Ok(())
}

/// Generate signing keypair
pub fn run_keygen() -> HorusResult<()> {
    registry::generate_signing_keypair()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
    Ok(())
}

/// Unpublish a package from the registry
pub fn run_unpublish(package: String, version: String, yes: bool) -> HorusResult<()> {
    use std::io::{self, Write};

    println!(
        "{} Unpublishing {} v{}...",
        cli_output::ICON_INFO.cyan(),
        package.yellow(),
        version.yellow()
    );

    // Confirmation prompt (unless --yes flag is set)
    if !yes {
        println!(
            "\n{} This action is {} and will:",
            "Warning:".yellow().bold(),
            "IRREVERSIBLE".red().bold()
        );
        println!("  • Delete {} v{} from the registry", package, version);
        println!("  • Make this version unavailable for download");
        println!("  • Cannot be undone");
        println!(
            "\n{} Consider using 'yank' instead for temporary removal",
            "Tip:".dimmed()
        );

        print!("\nType the package name '{}' to confirm: ", package);
        let _ = io::stdout().flush();

        let mut confirmation = String::new();
        io::stdin().read_line(&mut confirmation).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!("Failed to read input: {}", e)))
        })?;

        if confirmation.trim() != package {
            println!("  Package name mismatch. Unpublish cancelled.");
            return Ok(());
        }
    }

    // Call unpublish API
    let client = registry::RegistryClient::new();
    client
        .unpublish(&package, &version)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    println!(
        "\n  Successfully unpublished {} v{}",
        package.green(),
        version.green()
    );
    println!("  The package is no longer available on the registry");

    Ok(())
}

/// Add a dependency.
///
/// Detects the project language from CWD and delegates to the native package
/// manager (`cargo add` for Rust, `pip install` for Python).
pub fn run_add(name: String, ver: Option<String>, driver: bool, _plugin: bool) -> HorusResult<()> {
    use crate::manifest::DriverValue;

    // ── Write to horus.toml (single source of truth) ─────────────────────
    let manifest_path = Path::new(HORUS_TOML);
    if !manifest_path.exists() {
        return Err(HorusError::Config(ConfigError::Other(
            "No horus.toml found. Run `horus new` to create a project first.".to_string(),
        )));
    }

    let mut manifest = HorusManifest::load_from(manifest_path)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if driver {
        // Add as driver to [drivers] section
        let backend = ver.clone().unwrap_or_else(|| "true".to_string());
        let driver_value = if backend == "true" {
            DriverValue::Enabled(true)
        } else {
            DriverValue::Backend(backend)
        };
        manifest.drivers.insert(name.clone(), driver_value);
        manifest
            .save_to(manifest_path)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        println!(
            "{} Added driver {} to horus.toml [drivers]",
            cli_output::ICON_SUCCESS.green(),
            name.green(),
        );
    } else {
        // Auto-detect source from the dependency name using smart resolution.
        use crate::manifest::{DepSource, DependencyValue, DetailedDependency};
        use crate::source_resolver::{Confidence, PackageSourceResolver};

        let ctx = crate::dispatch::detect_context(&std::env::current_dir()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?);
        let resolver = PackageSourceResolver::new(&ctx.languages);
        let resolved = resolver.resolve_with_network(&name);

        // If no version specified, try to fetch the latest from the source.
        // Falls back to "*" if network is unavailable (3s timeout).
        let (version_string, fetched) = if let Some(ref v) = ver {
            (v.clone(), false)
        } else {
            match crate::source_resolver::fetch_latest_version(&name, &resolved.source) {
                Some(v) => (v, true),
                None => ("*".to_string(), false),
            }
        };

        let dep_value = match resolved.source {
            DepSource::CratesIo | DepSource::PyPI => {
                // Known external package — store with explicit source
                DependencyValue::Detailed(DetailedDependency {
                    version: Some(version_string.clone()),
                    source: Some(resolved.source.clone()),
                    features: vec![],
                    optional: false,
                    path: None,
                    git: None,
                    branch: None,
                    tag: None,
                    rev: None,
                    apt: None,
                    cmake_package: None,
                    lang: None,
                })
            }
            _ => {
                // Registry or unknown — use simple form (backward compatible)
                DependencyValue::Simple(version_string.clone())
            }
        };

        let was_present = manifest.dependencies.contains_key(&name);
        manifest.dependencies.insert(name.clone(), dep_value);
        manifest
            .save_to(manifest_path)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        let action = if was_present { "Updated" } else { "Added" };
        let version_display = if fetched {
            format!("@{} (latest)", version_string)
        } else if ver.is_some() {
            format!("@{}", version_string)
        } else {
            String::new()
        };

        // Show source info for transparency
        let source_hint = match resolved.source {
            DepSource::Registry => String::new(),
            _ => {
                let confidence_label = match resolved.confidence {
                    Confidence::High => "",
                    Confidence::Medium => " (inferred)",
                    Confidence::Low => " (guessed)",
                };
                format!(" [source: {}{}]", resolved.source, confidence_label)
            }
        };

        println!(
            "{} {} {}{} to horus.toml [dependencies]{}",
            cli_output::ICON_SUCCESS.green(),
            action,
            name.green(),
            version_display,
            source_hint.dimmed(),
        );
    }

    Ok(())
}

/// Remove a dependency.
///
/// Detects the project language from CWD and delegates to the native package
/// manager (`cargo remove` for Rust, `pip uninstall -y` for Python).
pub fn run_remove_dep(name: String) -> HorusResult<()> {
    // ── Remove from horus.toml first (single source of truth) ────────────
    let manifest_path = Path::new(HORUS_TOML);
    if manifest_path.exists() {
        let mut manifest = HorusManifest::load_from(manifest_path)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        let removed_from_deps = manifest.dependencies.remove(&name).is_some();
        let removed_from_dev = manifest.dev_dependencies.remove(&name).is_some();
        let removed_from_drivers = manifest.drivers.remove(&name).is_some();

        if removed_from_deps || removed_from_dev || removed_from_drivers {
            manifest
                .save_to(manifest_path)
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

            let sections: Vec<&str> = [
                if removed_from_deps { Some("[dependencies]") } else { None },
                if removed_from_dev { Some("[dev-dependencies]") } else { None },
                if removed_from_drivers { Some("[drivers]") } else { None },
            ]
            .into_iter()
            .flatten()
            .collect();

            println!(
                "{} Removed {} from horus.toml {}",
                cli_output::ICON_SUCCESS.green(),
                name.green(),
                sections.join(", "),
            );

            // ── Regenerate build files so .horus/ stays in sync ──────────
            let project_dir = std::env::current_dir()
                .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            let ctx = crate::dispatch::detect_context(&project_dir);

            let mut regenerated = Vec::new();

            if ctx.languages.contains(&crate::manifest::Language::Rust) {
                if let Ok(path) = crate::cargo_gen::generate(&manifest, &project_dir, &[], false) {
                    regenerated.push(path.display().to_string());
                }
            }

            if ctx.languages.contains(&crate::manifest::Language::Python) {
                if let Ok(path) = crate::pyproject_gen::generate(&manifest, &project_dir, false) {
                    regenerated.push(path.display().to_string());
                }
            }

            if !regenerated.is_empty() {
                println!(
                    "{} Regenerated build files",
                    cli_output::ICON_SUCCESS.green(),
                );
            }
        } else {
            println!(
                "{} {} not found in horus.toml dependencies, dev-dependencies, or drivers",
                cli_output::ICON_WARN.yellow(),
                name,
            );
        }
    } else {
        return Err(HorusError::Config(ConfigError::Other(
            "No horus.toml found. Run `horus new` to create a project first.".to_string(),
        )));
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    // ── detect_plugin_metadata ──────────────────────────────────────────

    #[test]
    fn test_detect_plugin_metadata_none() {
        let temp_dir = TempDir::new().unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_from_horus_toml_plugin_section() {
        let temp_dir = TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "horus-sim"
version = "0.3.0"

[plugin]
command = "sim"
binary = "bin/horus-sim"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();

        // Create the binary file so the path resolves
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        fs::write(bin_dir.join("horus-sim"), "#!/bin/sh\n").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.command, "sim");
        assert_eq!(meta.package_name, "horus-sim");
        assert_eq!(meta.version, "0.3.0");
    }

    #[test]
    fn test_detect_plugin_metadata_horus_toml_with_subcommands() {
        let temp_dir = TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "horus-deploy"
version = "1.0.0"

[plugin]
command = "deploy"
binary = "bin/horus-deploy"

[[plugin.subcommands]]
name = "start"
description = "Start deployment"

[[plugin.subcommands]]
name = "stop"
description = "Stop deployment"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        fs::write(bin_dir.join("horus-deploy"), "#!/bin/sh\n").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.command, "deploy");
        assert_eq!(meta.commands.len(), 2);
        assert_eq!(meta.commands[0].name, "start");
        assert_eq!(meta.commands[0].description, "Start deployment");
        assert_eq!(meta.commands[1].name, "stop");
    }

    #[test]
    fn test_detect_plugin_metadata_horus_toml_with_compatibility() {
        let temp_dir = TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "horus-ext"
version = "0.1.0"

[plugin]
command = "ext"
binary = "bin/horus-ext"

[plugin.compatibility]
horus = ">=0.5.0,<1.0.0"
platforms = ["linux-x86_64", "linux-aarch64"]
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        fs::write(bin_dir.join("horus-ext"), "#!/bin/sh\n").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.compatibility.horus_min, "0.5.0");
        assert_eq!(meta.compatibility.horus_max, "1.0.0");
        assert_eq!(meta.compatibility.platforms.len(), 2);
        assert!(meta.compatibility.platforms.contains(&"linux-x86_64".to_string()));
    }

    #[test]
    fn test_detect_plugin_metadata_horus_toml_with_permissions() {
        let temp_dir = TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "horus-net"
version = "0.2.0"

[plugin]
command = "net"
binary = "bin/horus-net"
permissions = ["network", "filesystem"]
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        fs::write(bin_dir.join("horus-net"), "#!/bin/sh\n").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.permissions, vec!["network", "filesystem"]);
    }

    #[test]
    fn test_detect_plugin_metadata_horus_toml_missing_command_field() {
        let temp_dir = TempDir::new().unwrap();
        // plugin section exists but no command field => None from parse_plugin_toml_manifest
        let toml_content = r#"
[package]
name = "horus-bad"
version = "0.1.0"

[plugin]
binary = "bin/horus-bad"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        fs::write(bin_dir.join("horus-bad"), "#!/bin/sh\n").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_horus_toml_missing_binary_field() {
        let temp_dir = TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "horus-bad"
version = "0.1.0"

[plugin]
command = "bad"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_invalid_toml() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(temp_dir.path().join(HORUS_TOML), "not valid {{{ toml").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_from_cargo_toml_metadata_section() {
        let temp_dir = TempDir::new().unwrap();
        let cargo_content = r#"
[package]
name = "horus-tools"
version = "2.0.0"

[package.metadata.horus]
cli_extension = true
command_name = "tools"
"#;
        fs::write(temp_dir.path().join(CARGO_TOML), cargo_content).unwrap();

        // parse_plugin_toml calls find_binary which needs the binary on disk
        // Without a binary, parse_plugin_toml returns None, so detect falls through
        let result = detect_plugin_metadata(temp_dir.path());
        // No binary means None for this path, but the auto-detect [[bin]] path
        // also won't match since package name is "horus-tools" and it needs a binary.
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_cargo_toml_cli_extension_false() {
        let temp_dir = TempDir::new().unwrap();
        let cargo_content = r#"
[package]
name = "my-lib"
version = "1.0.0"

[package.metadata.horus]
cli_extension = false
command_name = "mylib"
"#;
        fs::write(temp_dir.path().join(CARGO_TOML), cargo_content).unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_cargo_toml_no_horus_metadata() {
        let temp_dir = TempDir::new().unwrap();
        let cargo_content = r#"
[package]
name = "some-lib"
version = "0.5.0"
"#;
        fs::write(temp_dir.path().join(CARGO_TOML), cargo_content).unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_invalid_cargo_toml() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(temp_dir.path().join(CARGO_TOML), "garbage {{{{ not toml").unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[cfg(unix)]
    #[test]
    fn test_detect_plugin_metadata_from_bin_dir_horus_prefix() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();

        // Create an executable named horus-check
        let binary_path = bin_dir.join("horus-check");
        fs::write(&binary_path, "#!/bin/sh\necho check").unwrap();
        fs::set_permissions(&binary_path, fs::Permissions::from_mode(0o755)).unwrap();

        // Also write a horus.toml so detect_version can find a version
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"checker\"\nversion = \"0.9.0\"\n",
        )
        .unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.command, "check");
        assert_eq!(meta.version, "0.9.0");
        assert_eq!(meta.binary, binary_path);
    }

    #[cfg(unix)]
    #[test]
    fn test_detect_plugin_metadata_bin_dir_non_executable() {
        let temp_dir = TempDir::new().unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();

        // Create a non-executable file
        let binary_path = bin_dir.join("horus-notexec");
        fs::write(&binary_path, "#!/bin/sh\necho").unwrap();
        // default permissions are typically 0o644, not executable

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_bin_dir_no_horus_prefix() {
        let temp_dir = TempDir::new().unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();

        // File without horus- prefix
        fs::write(bin_dir.join("some-tool"), "#!/bin/sh").unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[cfg(unix)]
    #[test]
    fn test_detect_plugin_metadata_auto_detect_cargo_horus_pkg() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();

        // Cargo.toml with horus- prefixed package and [[bin]]
        let cargo_content = r#"
[package]
name = "horus-flash"
version = "3.0.0"
description = "Flash firmware tool"

[[bin]]
name = "flash"
path = "src/main.rs"
"#;
        fs::write(temp_dir.path().join(CARGO_TOML), cargo_content).unwrap();

        // Create the binary at target/debug/flash
        let debug_dir = temp_dir.path().join("target/debug");
        fs::create_dir_all(&debug_dir).unwrap();
        let binary = debug_dir.join("flash");
        fs::write(&binary, "#!/bin/sh").unwrap();
        fs::set_permissions(&binary, fs::Permissions::from_mode(0o755)).unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.command, "flash");
        assert_eq!(meta.package_name, "horus-flash");
        assert_eq!(meta.version, "3.0.0");
        assert_eq!(meta.commands.len(), 1);
        assert_eq!(meta.commands[0].name, "flash");
        assert_eq!(meta.commands[0].description, "Flash firmware tool");
    }

    #[test]
    fn test_detect_plugin_metadata_auto_detect_non_horus_pkg() {
        let temp_dir = TempDir::new().unwrap();
        let cargo_content = r#"
[package]
name = "not-a-horus-pkg"
version = "1.0.0"

[[bin]]
name = "mybin"
path = "src/main.rs"
"#;
        fs::write(temp_dir.path().join(CARGO_TOML), cargo_content).unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[cfg(unix)]
    #[test]
    fn test_detect_plugin_metadata_auto_detect_fallback_package_name_as_command() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        // Cargo.toml with horus- prefix but no [[bin]] section
        let cargo_content = r#"
[package]
name = "horus-monitor"
version = "1.5.0"
"#;
        fs::write(temp_dir.path().join(CARGO_TOML), cargo_content).unwrap();

        // Command name falls back to stripping "horus-" => "monitor"
        // Create a binary named "monitor" in target/release
        let release_dir = temp_dir.path().join("target/release");
        fs::create_dir_all(&release_dir).unwrap();
        let binary = release_dir.join("monitor");
        fs::write(&binary, "#!/bin/sh").unwrap();
        fs::set_permissions(&binary, fs::Permissions::from_mode(0o755)).unwrap();

        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.command, "monitor");
        assert_eq!(meta.package_name, "horus-monitor");
    }

    // ── detect_version ──────────────────────────────────────────────────

    #[test]
    fn test_detect_version_from_horus_toml() {
        let temp_dir = TempDir::new().unwrap();
        let toml_path = temp_dir.path().join(HORUS_TOML);
        fs::write(
            &toml_path,
            "[package]\nname = \"test-pkg\"\nversion = \"1.2.3\"\n",
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("1.2.3".to_string()));
    }

    #[test]
    fn test_detect_version_from_cargo_toml() {
        let temp_dir = TempDir::new().unwrap();
        let cargo_path = temp_dir.path().join(CARGO_TOML);
        fs::write(
            &cargo_path,
            "[package]\nname = \"test-crate\"\nversion = \"4.5.6\"\n",
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("4.5.6".to_string()));
    }

    #[test]
    fn test_detect_version_from_metadata_json() {
        let temp_dir = TempDir::new().unwrap();
        let json_path = temp_dir.path().join("metadata.json");
        fs::write(&json_path, r#"{"version": "7.8.9"}"#).unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("7.8.9".to_string()));
    }

    #[test]
    fn test_detect_version_priority_horus_over_cargo() {
        let temp_dir = TempDir::new().unwrap();
        // horus.toml should take priority over Cargo.toml
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"pkg\"\nversion = \"1.0.0\"\n",
        )
        .unwrap();
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"pkg\"\nversion = \"2.0.0\"\n",
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("1.0.0".to_string()));
    }

    #[test]
    fn test_detect_version_priority_cargo_over_json() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"pkg\"\nversion = \"2.0.0\"\n",
        )
        .unwrap();
        fs::write(
            temp_dir.path().join("metadata.json"),
            r#"{"version": "3.0.0"}"#,
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("2.0.0".to_string()));
    }

    #[test]
    fn test_detect_version_none_empty_dir() {
        let temp_dir = TempDir::new().unwrap();
        let version = detect_version(temp_dir.path());
        assert_eq!(version, None);
    }

    #[test]
    fn test_detect_version_invalid_horus_toml() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(temp_dir.path().join(HORUS_TOML), "not valid toml {{{").unwrap();
        // Falls through to Cargo.toml, which doesn't exist => None
        let version = detect_version(temp_dir.path());
        assert_eq!(version, None);
    }

    #[test]
    fn test_detect_version_cargo_toml_no_version() {
        let temp_dir = TempDir::new().unwrap();
        // Cargo.toml exists but has no version field
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"no-ver\"\n",
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, None);
    }

    #[test]
    fn test_detect_version_metadata_json_no_version() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(
            temp_dir.path().join("metadata.json"),
            r#"{"name": "no-ver"}"#,
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, None);
    }

    #[test]
    fn test_detect_version_invalid_metadata_json() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(temp_dir.path().join("metadata.json"), "not json at all").unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, None);
    }

    // ── find_binary ─────────────────────────────────────────────────────

    #[cfg(unix)]
    #[test]
    fn test_find_binary_in_bin_dir() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        let bin_path = bin_dir.join("my-tool");
        fs::write(&bin_path, "#!/bin/sh").unwrap();
        fs::set_permissions(&bin_path, fs::Permissions::from_mode(0o755)).unwrap();

        let result = find_binary(temp_dir.path(), "my-tool");
        assert_eq!(result, Some(bin_path));
    }

    #[cfg(unix)]
    #[test]
    fn test_find_binary_in_target_release() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        let release_dir = temp_dir.path().join("target/release");
        fs::create_dir_all(&release_dir).unwrap();
        let bin_path = release_dir.join("my-tool");
        fs::write(&bin_path, "#!/bin/sh").unwrap();
        fs::set_permissions(&bin_path, fs::Permissions::from_mode(0o755)).unwrap();

        let result = find_binary(temp_dir.path(), "my-tool");
        assert_eq!(result, Some(bin_path));
    }

    #[cfg(unix)]
    #[test]
    fn test_find_binary_in_target_debug() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        let debug_dir = temp_dir.path().join("target/debug");
        fs::create_dir_all(&debug_dir).unwrap();
        let bin_path = debug_dir.join("my-tool");
        fs::write(&bin_path, "#!/bin/sh").unwrap();
        fs::set_permissions(&bin_path, fs::Permissions::from_mode(0o755)).unwrap();

        let result = find_binary(temp_dir.path(), "my-tool");
        assert_eq!(result, Some(bin_path));
    }

    #[cfg(unix)]
    #[test]
    fn test_find_binary_in_package_dir_root() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        let bin_path = temp_dir.path().join("my-tool");
        fs::write(&bin_path, "#!/bin/sh").unwrap();
        fs::set_permissions(&bin_path, fs::Permissions::from_mode(0o755)).unwrap();

        let result = find_binary(temp_dir.path(), "my-tool");
        assert_eq!(result, Some(bin_path));
    }

    #[cfg(unix)]
    #[test]
    fn test_find_binary_prefers_bin_dir_over_target() {
        use std::os::unix::fs::PermissionsExt;

        let temp_dir = TempDir::new().unwrap();
        // Put binary in both bin/ and target/release/ — bin/ should be returned
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        let bin_path = bin_dir.join("tool");
        fs::write(&bin_path, "#!/bin/sh").unwrap();
        fs::set_permissions(&bin_path, fs::Permissions::from_mode(0o755)).unwrap();

        let release_dir = temp_dir.path().join("target/release");
        fs::create_dir_all(&release_dir).unwrap();
        let release_bin = release_dir.join("tool");
        fs::write(&release_bin, "#!/bin/sh").unwrap();
        fs::set_permissions(&release_bin, fs::Permissions::from_mode(0o755)).unwrap();

        let result = find_binary(temp_dir.path(), "tool");
        assert_eq!(result, Some(bin_path));
    }

    #[test]
    fn test_find_binary_not_found() {
        let temp_dir = TempDir::new().unwrap();
        let result = find_binary(temp_dir.path(), "nonexistent");
        assert_eq!(result, None);
    }

    #[test]
    fn test_find_binary_file_exists_but_not_executable() {
        let temp_dir = TempDir::new().unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        // Write a non-executable file
        fs::write(bin_dir.join("my-tool"), "not executable").unwrap();

        // On unix, default permissions are 0o644 (not executable)
        // On non-unix, is_executable just checks existence, so this may pass
        #[cfg(unix)]
        {
            let result = find_binary(temp_dir.path(), "my-tool");
            assert_eq!(result, None);
        }
    }

    // ── read_package_name_from_path ─────────────────────────────────────

    #[test]
    fn test_read_package_name_from_horus_toml() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"my-horus-pkg\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let name = read_package_name_from_path(temp_dir.path()).unwrap();
        assert_eq!(name, "my-horus-pkg");
    }

    #[test]
    fn test_read_package_name_from_cargo_toml() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"my-crate\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let name = read_package_name_from_path(temp_dir.path()).unwrap();
        assert_eq!(name, "my-crate");
    }

    #[test]
    fn test_read_package_name_horus_toml_priority_over_cargo() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"from-horus\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"from-cargo\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let name = read_package_name_from_path(temp_dir.path()).unwrap();
        assert_eq!(name, "from-horus");
    }

    #[test]
    fn test_read_package_name_fallback_to_dir_name() {
        let temp_dir = TempDir::new().unwrap();
        let sub_dir = temp_dir.path().join("my-awesome-package");
        fs::create_dir_all(&sub_dir).unwrap();
        // No manifests => falls back to directory name
        let name = read_package_name_from_path(&sub_dir).unwrap();
        assert_eq!(name, "my-awesome-package");
    }

    #[test]
    fn test_read_package_name_cargo_toml_no_name_field() {
        let temp_dir = TempDir::new().unwrap();
        // Cargo.toml without a name field
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Falls back to directory name
        let name = read_package_name_from_path(temp_dir.path()).unwrap();
        // The temp dir name is random, just verify it's not empty
        assert!(!name.is_empty());
    }

    // ── resolve_installed_package_dir ────────────────────────────────────

    #[test]
    fn test_resolve_installed_package_dir_global_versioned() {
        let temp_dir = TempDir::new().unwrap();
        let cache_dir = temp_dir.path().join(".horus/cache");
        let versioned_dir = cache_dir.join("my-pkg@1.0.0");
        fs::create_dir_all(&versioned_dir).unwrap();

        // This test is tricky because it uses dirs::home_dir() internally.
        // We can only test the non-global (local) path without mocking HOME.
        // The global path always uses the real home directory.
        // We verify the function doesn't panic for non-existent paths.
        let result = resolve_installed_package_dir("nonexistent-pkg", "1.0.0", true);
        assert!(result.is_none());
    }

    #[test]
    fn test_resolve_installed_package_dir_local_not_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let result = resolve_installed_package_dir("nonexistent-pkg", "1.0.0", false);
        assert!(result.is_none());

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ── print_package_info ──────────────────────────────────────────────

    #[test]
    fn test_print_package_info_path_metadata() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("my-pkg");
        fs::create_dir_all(&entry_path).unwrap();

        // Create path metadata file
        fs::write(
            packages_dir.join("my-pkg.path.json"),
            r#"{"version": "dev", "source_path": "/some/path"}"#,
        )
        .unwrap();

        let printed = print_package_info(packages_dir, "my-pkg", &entry_path);
        assert!(printed);
    }

    #[test]
    fn test_print_package_info_system_metadata() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("sys-pkg");
        fs::create_dir_all(&entry_path).unwrap();

        fs::write(
            packages_dir.join("sys-pkg.system.json"),
            r#"{"version": "2.0.0"}"#,
        )
        .unwrap();

        let printed = print_package_info(packages_dir, "sys-pkg", &entry_path);
        assert!(printed);
    }

    #[test]
    fn test_print_package_info_registry_metadata() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("reg-pkg");
        fs::create_dir_all(&entry_path).unwrap();

        // Create metadata.json inside the package directory
        fs::write(
            entry_path.join("metadata.json"),
            r#"{"version": "1.0.0"}"#,
        )
        .unwrap();

        let printed = print_package_info(packages_dir, "reg-pkg", &entry_path);
        assert!(printed);
    }

    #[test]
    fn test_print_package_info_no_metadata() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("bare-pkg");
        fs::create_dir_all(&entry_path).unwrap();

        let printed = print_package_info(packages_dir, "bare-pkg", &entry_path);
        assert!(!printed);
    }

    #[test]
    fn test_print_package_info_invalid_json_files() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("bad-pkg");
        fs::create_dir_all(&entry_path).unwrap();

        // Invalid JSON in all metadata files
        fs::write(packages_dir.join("bad-pkg.path.json"), "not json").unwrap();
        fs::write(packages_dir.join("bad-pkg.system.json"), "not json").unwrap();
        fs::write(entry_path.join("metadata.json"), "not json").unwrap();

        let printed = print_package_info(packages_dir, "bad-pkg", &entry_path);
        assert!(!printed);
    }

    #[test]
    fn test_print_package_info_priority_path_over_system() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("multi-pkg");
        fs::create_dir_all(&entry_path).unwrap();

        // Both path and system metadata exist
        fs::write(
            packages_dir.join("multi-pkg.path.json"),
            r#"{"version": "dev", "source_path": "/path"}"#,
        )
        .unwrap();
        fs::write(
            packages_dir.join("multi-pkg.system.json"),
            r#"{"version": "1.0"}"#,
        )
        .unwrap();

        // path.json takes priority (checked first)
        let printed = print_package_info(packages_dir, "multi-pkg", &entry_path);
        assert!(printed);
    }

    // ── remove_from_horus_toml ──────────────────────────────────────────

    #[test]
    fn test_remove_from_horus_toml_removes_dependency() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let manifest_content = r#"[package]
name = "test-proj"
version = "0.1.0"

[dependencies]
rplidar = "1.0.0"
other-dep = "2.0.0"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), manifest_content).unwrap();

        remove_from_horus_toml("rplidar");

        // Verify rplidar was removed
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("rplidar"));
        assert!(manifest.dependencies.contains_key("other-dep"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_remove_from_horus_toml_removes_dev_dependency() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let manifest_content = r#"[package]
name = "test-proj"
version = "0.1.0"

[dev-dependencies]
test-helper = "1.0.0"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), manifest_content).unwrap();

        remove_from_horus_toml("test-helper");

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dev_dependencies.contains_key("test-helper"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_remove_from_horus_toml_nonexistent_package() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let manifest_content = r#"[package]
name = "test-proj"
version = "0.1.0"

[dependencies]
existing = "1.0.0"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), manifest_content).unwrap();

        // Should not panic when removing a package that doesn't exist
        remove_from_horus_toml("nonexistent");

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("existing"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_remove_from_horus_toml_no_manifest_file() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Should not panic when no horus.toml exists
        remove_from_horus_toml("anything");

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ── remove_from_horus_lock ──────────────────────────────────────────

    #[test]
    fn test_remove_from_horus_lock_removes_package() {
        use crate::lockfile::{HorusLockfile, LockedPackage, HORUS_LOCK};

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let mut lockfile = HorusLockfile::new();
        lockfile.packages = vec![
            LockedPackage {
                name: "keep-me".to_string(),
                version: "1.0.0".to_string(),
                source: "registry".to_string(),
                checksum: None,
            },
            LockedPackage {
                name: "remove-me".to_string(),
                version: "2.0.0".to_string(),
                source: "registry".to_string(),
                checksum: None,
            },
        ];
        lockfile
            .save_to(std::path::Path::new(HORUS_LOCK))
            .unwrap();

        remove_from_horus_lock("remove-me");

        let loaded = HorusLockfile::load_from(std::path::Path::new(HORUS_LOCK)).unwrap();
        assert_eq!(loaded.packages.len(), 1);
        assert_eq!(loaded.packages[0].name, "keep-me");

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_remove_from_horus_lock_no_lockfile() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Should not panic when no lockfile exists
        remove_from_horus_lock("anything");

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_remove_from_horus_lock_package_not_in_lockfile() {
        use crate::lockfile::{HorusLockfile, LockedPackage, HORUS_LOCK};

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let mut lockfile = HorusLockfile::new();
        lockfile.packages = vec![LockedPackage {
            name: "existing".to_string(),
            version: "1.0.0".to_string(),
            source: "registry".to_string(),
            checksum: None,
        }];
        lockfile
            .save_to(std::path::Path::new(HORUS_LOCK))
            .unwrap();

        remove_from_horus_lock("nonexistent");

        let loaded = HorusLockfile::load_from(std::path::Path::new(HORUS_LOCK)).unwrap();
        assert_eq!(loaded.packages.len(), 1);
        assert_eq!(loaded.packages[0].name, "existing");

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ── PluginMetadata struct ───────────────────────────────────────────

    #[test]
    fn test_plugin_metadata_clone() {
        let meta = PluginMetadata {
            command: "test".to_string(),
            binary: PathBuf::from("/usr/bin/test"),
            package_name: "horus-test".to_string(),
            version: "1.0.0".to_string(),
            commands: vec![CommandInfo {
                name: "run".to_string(),
                description: "Run tests".to_string(),
            }],
            compatibility: Compatibility::default(),
            permissions: vec!["network".to_string()],
        };

        let cloned = meta.clone();
        assert_eq!(cloned.command, "test");
        assert_eq!(cloned.binary, PathBuf::from("/usr/bin/test"));
        assert_eq!(cloned.package_name, "horus-test");
        assert_eq!(cloned.version, "1.0.0");
        assert_eq!(cloned.commands.len(), 1);
        assert_eq!(cloned.permissions, vec!["network"]);
    }

    #[test]
    fn test_plugin_metadata_debug() {
        let meta = PluginMetadata {
            command: "dbg".to_string(),
            binary: PathBuf::from("/bin/horus-dbg"),
            package_name: "horus-dbg".to_string(),
            version: "0.1.0".to_string(),
            commands: vec![],
            compatibility: Compatibility::default(),
            permissions: vec![],
        };

        let debug_str = format!("{:?}", meta);
        assert!(debug_str.contains("dbg"));
        assert!(debug_str.contains("horus-dbg"));
    }

    // ── run_install source detection ────────────────────────────────────

    #[test]
    fn test_run_install_detects_path_source_from_slash() {
        // Tests the source detection logic:
        // package containing "/" should be treated as path
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // No horus.toml => error, but the source detection itself works
        let result = run_install(
            "./local/pkg".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        );
        // Should error about missing horus.toml (not about source detection)
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(
            err_msg.contains("horus.toml"),
            "Expected horus.toml error, got: {}",
            err_msg
        );

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_detects_git_source() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let result = run_install(
            "https://github.com/user/repo.git".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        );
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("horus.toml"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_with_invalid_source_flag() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let result = run_install(
            "some-pkg".to_string(),
            None,
            false,
            None,
            Some("invalid-source".to_string()),
            None,
            false,
        );
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("Unknown source"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_source_flag_crates_io_variants() {
        // All crates.io aliases should be accepted; they all fail at manifest
        // loading since there's no horus.toml, which proves the source was parsed.
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        for alias in &["crates.io", "crates", "cargo"] {
            let result = run_install(
                "serde".to_string(),
                None,
                false,
                None,
                Some(alias.to_string()),
                None,
                false,
            );
            assert!(result.is_err(), "Expected error for alias '{}'", alias);
            let err_msg = format!("{}", result.unwrap_err());
            assert!(
                err_msg.contains("horus.toml"),
                "Alias '{}' should fail at manifest, got: {}",
                alias,
                err_msg
            );
        }

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_source_flag_pypi_variants() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        for alias in &["pypi", "pip", "python"] {
            let result = run_install(
                "numpy".to_string(),
                None,
                false,
                None,
                Some(alias.to_string()),
                None,
                false,
            );
            assert!(result.is_err());
            let err_msg = format!("{}", result.unwrap_err());
            assert!(err_msg.contains("horus.toml"));
        }

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_source_flag_system_variants() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        for alias in &["system", "apt", "brew"] {
            let result = run_install(
                "curl".to_string(),
                None,
                false,
                None,
                Some(alias.to_string()),
                None,
                false,
            );
            assert!(result.is_err());
        }

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_source_flag_registry_variants() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        for alias in &["registry", "horus"] {
            let result = run_install(
                "some-pkg".to_string(),
                None,
                false,
                None,
                Some(alias.to_string()),
                None,
                false,
            );
            assert!(result.is_err());
        }

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_path_source_nonexistent_path() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Create horus.toml so we get past the manifest check
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "./nonexistent/path".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        );
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("Path does not exist"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_path_source_existing_path() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Create horus.toml for the project
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-proj\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Create a local package directory with its own manifest
        let local_pkg = temp_dir.path().join("local-dep");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"local-dep\"\nversion = \"0.2.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "./local-dep".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        );
        // Should succeed — path dependency gets written to horus.toml
        assert!(result.is_ok(), "run_install failed: {:?}", result.err());

        // Verify the dependency was added to horus.toml
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("local-dep"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_git_dep_writes_to_manifest() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-proj\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Must use --source git because URL contains '/' which triggers is_path detection
        let result = run_install(
            "https://github.com/user/my-pkg.git".to_string(),
            None,
            false,
            None,
            Some("git".to_string()),
            None,
            false,
        );

        // Restore CWD before any assertions to prevent CWD leak on panic
        std::env::set_current_dir(&original_dir).unwrap();

        assert!(result.is_ok(), "run_install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        // Git dep name derived from URL: "my-pkg" (strips .git suffix)
        assert!(
            manifest.dependencies.contains_key("my-pkg"),
            "Expected 'my-pkg' in deps, found: {:?}",
            manifest.dependencies.keys().collect::<Vec<_>>()
        );
    }

    #[test]
    fn test_run_install_dev_flag_adds_to_dev_deps() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-proj\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let local_pkg = temp_dir.path().join("test-util");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"test-util\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "./test-util".to_string(),
            None,
            false,
            None,
            None,
            None,
            true, // dev = true
        );
        assert!(result.is_ok(), "run_install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dev_dependencies.contains_key("test-util"),
            "Expected 'test-util' in dev-deps"
        );
        assert!(
            !manifest.dependencies.contains_key("test-util"),
            "'test-util' should not be in regular deps"
        );

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_crates_io_with_version_and_features() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-proj\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "serde".to_string(),
            Some("1.0.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        );
        assert!(result.is_ok(), "run_install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("serde"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_no_manifest_errors() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let result = run_install(
            "some-pkg".to_string(),
            None,
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        );
        assert!(result.is_err());
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("horus.toml"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    #[test]
    fn test_run_install_update_existing_dep() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let manifest_content = r#"[package]
name = "test-proj"
version = "0.1.0"

[dependencies]
my-pkg = "1.0.0"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), manifest_content).unwrap();

        let local_pkg = temp_dir.path().join("my-pkg");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"my-pkg\"\nversion = \"2.0.0\"\n",
        )
        .unwrap();

        // Re-install with path source replaces the existing entry
        let result = run_install(
            "./my-pkg".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        );
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("my-pkg"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ── run_install path detection helpers ───────────────────────────────

    #[test]
    fn test_path_detection_logic() {
        // Test the is_path detection logic used in run_install
        let test_cases = vec![
            ("./local", true),
            ("../sibling", true),
            ("~/my-pkg", true),
            ("/absolute/path", true),
            ("path/to/pkg", true),
            ("simple-name", false),
            ("serde", false),
        ];

        for (input, expected_is_path) in test_cases {
            let is_path = input.contains('/')
                || input.starts_with('.')
                || input.starts_with('~');
            assert_eq!(
                is_path, expected_is_path,
                "Path detection for '{}': expected {}, got {}",
                input, expected_is_path, is_path
            );
        }
    }

    #[test]
    fn test_git_detection_logic() {
        let test_cases = vec![
            ("https://github.com/user/repo", true),
            ("https://gitlab.com/user/repo.git", true),
            ("git://example.com/repo", true),
            ("http://example.com/repo", false),
            ("simple-name", false),
            ("./local", false),
        ];

        for (input, expected_is_git) in test_cases {
            let is_git = input.starts_with("https://") || input.starts_with("git://");
            assert_eq!(
                is_git, expected_is_git,
                "Git detection for '{}': expected {}, got {}",
                input, expected_is_git, is_git
            );
        }
    }

    // ── git dep name extraction ─────────────────────────────────────────

    #[test]
    fn test_git_dep_name_extraction() {
        let cases = vec![
            ("https://github.com/user/my-repo.git", "my-repo"),
            ("https://github.com/user/my-repo", "my-repo"),
            ("git://example.com/toolbox.git", "toolbox"),
            ("https://github.com/user/a", "a"),
        ];

        for (url, expected_name) in cases {
            let dep_name = url
                .rsplit('/')
                .next()
                .unwrap_or(url)
                .trim_end_matches(".git")
                .to_string();
            assert_eq!(
                dep_name, expected_name,
                "Git name extraction for '{}': expected '{}', got '{}'",
                url, expected_name, dep_name
            );
        }
    }

    // ── parse_plugin_toml (Cargo metadata) ──────────────────────────────

    #[test]
    fn test_parse_plugin_toml_cli_extension_missing() {
        let metadata: toml::Value = toml::from_str(r#"command_name = "test""#).unwrap();
        let toml: toml::Table = toml::from_str(
            r#"[package]
name = "test"
version = "0.1.0""#,
        )
        .unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml(&metadata, &toml, temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_plugin_toml_cli_extension_not_bool() {
        let metadata: toml::Value =
            toml::from_str(r#"cli_extension = "yes""#).unwrap();
        let toml: toml::Table = toml::from_str(
            r#"[package]
name = "test"
version = "0.1.0""#,
        )
        .unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml(&metadata, &toml, temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_parse_plugin_toml_missing_command_name() {
        let metadata: toml::Value =
            toml::from_str(r#"cli_extension = true"#).unwrap();
        let toml: toml::Table = toml::from_str(
            r#"[package]
name = "test"
version = "0.1.0""#,
        )
        .unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml(&metadata, &toml, temp_dir.path());
        assert!(result.is_none());
    }

    // ── parse_plugin_toml_manifest ──────────────────────────────────────

    #[test]
    fn test_parse_plugin_toml_manifest_minimal() {
        let plugin: toml::Value = toml::from_str(
            r#"
command = "test-cmd"
binary = "bin/horus-test"
"#,
        )
        .unwrap();

        let toml_table: toml::Table = toml::from_str(
            r#"
[package]
name = "my-plugin"
version = "1.2.3"
"#,
        )
        .unwrap();

        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml_manifest(&plugin, &toml_table, temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.command, "test-cmd");
        assert_eq!(meta.package_name, "my-plugin");
        assert_eq!(meta.version, "1.2.3");
        assert_eq!(meta.binary, temp_dir.path().join("bin/horus-test"));
        assert!(meta.commands.is_empty());
        assert!(meta.permissions.is_empty());
    }

    #[test]
    fn test_parse_plugin_toml_manifest_defaults_when_package_missing() {
        let plugin: toml::Value = toml::from_str(
            r#"
command = "x"
binary = "bin/x"
"#,
        )
        .unwrap();

        // No [package] section at all
        let toml_table: toml::Table = toml::from_str("").unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml_manifest(&plugin, &toml_table, temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.package_name, "unknown");
        assert_eq!(meta.version, "0.0.0");
    }

    #[test]
    fn test_parse_plugin_toml_manifest_compatibility_defaults() {
        let plugin: toml::Value = toml::from_str(
            r#"
command = "y"
binary = "bin/y"

[compatibility]
horus = ">=1.0.0"
"#,
        )
        .unwrap();

        let toml_table: toml::Table = toml::from_str("").unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml_manifest(&plugin, &toml_table, temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        // Only min specified, max should default to "2.0.0"
        assert_eq!(meta.compatibility.horus_min, "1.0.0");
        assert_eq!(meta.compatibility.horus_max, "2.0.0");
        assert!(meta.compatibility.platforms.is_empty());
    }

    #[test]
    fn test_parse_plugin_toml_manifest_no_compatibility() {
        let plugin: toml::Value = toml::from_str(
            r#"
command = "z"
binary = "bin/z"
"#,
        )
        .unwrap();

        let toml_table: toml::Table = toml::from_str("").unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml_manifest(&plugin, &toml_table, temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        // Should use Compatibility::default()
        let default_compat = Compatibility::default();
        assert_eq!(meta.compatibility.horus_min, default_compat.horus_min);
        assert_eq!(meta.compatibility.horus_max, default_compat.horus_max);
    }

    // ── Edge cases and misc ─────────────────────────────────────────────

    #[test]
    fn test_detect_plugin_metadata_empty_bin_dir() {
        let temp_dir = TempDir::new().unwrap();
        let bin_dir = temp_dir.path().join("bin");
        fs::create_dir_all(&bin_dir).unwrap();
        // bin/ exists but is empty => no plugins detected
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_plugin_metadata_horus_toml_no_plugin_section() {
        let temp_dir = TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "regular-pkg"
version = "1.0.0"
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), toml_content).unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_version_empty_version_string() {
        let temp_dir = TempDir::new().unwrap();
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"pkg\"\nversion = \"\"\n",
        )
        .unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("".to_string()));
    }

    #[test]
    fn test_read_package_name_invalid_horus_toml_falls_to_cargo() {
        let temp_dir = TempDir::new().unwrap();
        // Invalid horus.toml
        fs::write(temp_dir.path().join(HORUS_TOML), "invalid {{").unwrap();
        // Valid Cargo.toml
        fs::write(
            temp_dir.path().join(CARGO_TOML),
            "[package]\nname = \"from-cargo\"\nversion = \"1.0.0\"\n",
        )
        .unwrap();

        let name = read_package_name_from_path(temp_dir.path()).unwrap();
        assert_eq!(name, "from-cargo");
    }

    #[test]
    fn test_print_package_info_missing_version_field() {
        let temp_dir = TempDir::new().unwrap();
        let packages_dir = temp_dir.path();
        let entry_path = packages_dir.join("pkg");
        fs::create_dir_all(&entry_path).unwrap();

        // metadata.json with no version field — should print "unknown"
        fs::write(entry_path.join("metadata.json"), r#"{"name": "pkg"}"#).unwrap();

        let printed = print_package_info(packages_dir, "pkg", &entry_path);
        assert!(printed);
    }

    #[test]
    fn test_parse_plugin_toml_manifest_subcommands_missing_name() {
        let plugin: toml::Value = toml::from_str(
            r#"
command = "test"
binary = "bin/test"

[[subcommands]]
description = "no name field here"

[[subcommands]]
name = "valid"
description = "this one is valid"
"#,
        )
        .unwrap();

        let toml_table: toml::Table = toml::from_str("").unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml_manifest(&plugin, &toml_table, temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        // Only the valid subcommand should be included
        assert_eq!(meta.commands.len(), 1);
        assert_eq!(meta.commands[0].name, "valid");
    }

    #[test]
    fn test_parse_plugin_toml_manifest_subcommand_no_description() {
        let plugin: toml::Value = toml::from_str(
            r#"
command = "test"
binary = "bin/test"

[[subcommands]]
name = "minimal"
"#,
        )
        .unwrap();

        let toml_table: toml::Table = toml::from_str("").unwrap();
        let temp_dir = TempDir::new().unwrap();

        let result = parse_plugin_toml_manifest(&plugin, &toml_table, temp_dir.path());
        assert!(result.is_some());
        let meta = result.unwrap();
        assert_eq!(meta.commands.len(), 1);
        assert_eq!(meta.commands[0].name, "minimal");
        assert_eq!(meta.commands[0].description, "");
    }

    #[test]
    fn test_run_install_registry_simple_form_no_version() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Registry source with no version and no features => Simple("*")
        let _result = run_install(
            "rplidar".to_string(),
            None,
            false,
            None,
            Some("registry".to_string()),
            None,
            false,
        );
        // This will fail at physical installation (RegistryClient), but
        // the manifest should have been written.
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("rplidar"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ── Battle-testing: run_add writes to horus.toml ─────────────────────

    #[test]
    fn test_run_add_writes_dep_to_horus_toml() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_add("my-sensor".to_string(), Some("1.2.0".to_string()), false, false);
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok(), "run_add should succeed: {:?}", result);

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("my-sensor"));
        assert_eq!(manifest.dependencies["my-sensor"].version(), Some("1.2.0"));
    }

    #[test]
    fn test_run_add_no_version_fetches_latest() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_add("rplidar".to_string(), None, false, false);
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("rplidar"));
        // Version auto-fetch queries PyPI/crates.io for latest version.
        // Falls back to "*" only when network is unavailable.
        let ver = manifest.dependencies["rplidar"].version();
        assert!(ver.is_some(), "version should be set");
        let v = ver.unwrap();
        assert!(v == "*" || v.contains('.'), "expected semver or wildcard fallback, got: {}", v);
    }

    #[test]
    fn test_run_add_driver_writes_to_drivers_section() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_add("camera".to_string(), Some("opencv".to_string()), true, false);
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.drivers.contains_key("camera"));
        assert!(!manifest.dependencies.contains_key("camera"), "driver should not appear in deps");
    }

    #[test]
    fn test_run_add_driver_boolean_form() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // No version for driver = boolean true form
        let result = run_add("gps".to_string(), None, true, false);
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.drivers.contains_key("gps"));
    }

    #[test]
    fn test_run_add_updates_existing_dep() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\nmy-sensor = \"1.0.0\"\n",
        )
        .unwrap();

        let result = run_add("my-sensor".to_string(), Some("2.0.0".to_string()), false, false);
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["my-sensor"].version(), Some("2.0.0"));
    }

    #[test]
    fn test_run_add_no_manifest_errors() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // No horus.toml exists
        let result = run_add("foo".to_string(), None, false, false);
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_err());
    }

    // ── Battle-testing: run_remove_dep removes from horus.toml ───────────

    #[test]
    fn test_run_remove_dep_removes_from_dependencies() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\nmy-sensor = \"1.0.0\"\nother-lib = \"2.0\"\n",
        )
        .unwrap();

        let result = run_remove_dep("my-sensor".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("my-sensor"), "dep should be removed");
        assert!(manifest.dependencies.contains_key("other-lib"), "other dep should remain");
    }

    #[test]
    fn test_run_remove_dep_removes_from_dev_dependencies() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n\n[dev-dependencies]\ncriterion = { version = \"0.5\", source = \"crates.io\" }\n",
        )
        .unwrap();

        let result = run_remove_dep("criterion".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dev_dependencies.contains_key("criterion"));
    }

    #[test]
    fn test_run_remove_dep_removes_from_drivers() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n\n[drivers]\ncamera = \"opencv\"\nlidar = \"rplidar-a2\"\n",
        )
        .unwrap();

        let result = run_remove_dep("camera".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.drivers.contains_key("camera"));
        assert!(manifest.drivers.contains_key("lidar"), "lidar driver should remain");
    }

    #[test]
    fn test_run_remove_dep_nonexistent_warns_gracefully() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Should succeed (warn but not error) when dep doesn't exist
        let result = run_remove_dep("nonexistent-pkg".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());
    }

    #[test]
    fn test_run_remove_dep_no_manifest_errors() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let result = run_remove_dep("foo".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_err());
    }

    #[test]
    fn test_run_remove_dep_regenerates_cargo_toml() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Create a Rust project with two deps
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"regen-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\nserde = { version = \"1.0\", source = \"crates.io\" }\ntokio = { version = \"1\", source = \"crates.io\" }\n",
        )
        .unwrap();

        // Create src/main.rs so language detection picks up Rust
        fs::create_dir_all(temp_dir.path().join("src")).unwrap();
        fs::write(temp_dir.path().join("src/main.rs"), "fn main() {}\n").unwrap();

        // Pre-generate .horus/Cargo.toml
        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        let cargo_path = crate::cargo_gen::generate(&manifest, temp_dir.path(), &[], false).unwrap();
        let cargo_content = fs::read_to_string(&cargo_path).unwrap();
        assert!(cargo_content.contains("serde"), "serde should be in Cargo.toml before removal");
        assert!(cargo_content.contains("tokio"), "tokio should be in Cargo.toml before removal");

        // Remove serde — should regenerate .horus/Cargo.toml without serde
        run_remove_dep("serde".to_string()).unwrap();
        std::env::set_current_dir(&original_dir).unwrap();

        let cargo_after = fs::read_to_string(&cargo_path).unwrap();
        assert!(!cargo_after.contains("serde"), "serde should be gone from Cargo.toml after removal");
        assert!(cargo_after.contains("tokio"), "tokio should remain in Cargo.toml");
    }

    #[test]
    fn test_run_remove_dep_regenerates_pyproject_toml() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Create a Python project with two deps
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"regen-py\"\nversion = \"0.1.0\"\n\n[dependencies]\nnumpy = { version = \">=1.24\", source = \"pypi\" }\nrequests = { version = \">=2.28\", source = \"pypi\" }\n",
        )
        .unwrap();

        // Create a .py file so language detection picks up Python
        fs::create_dir_all(temp_dir.path().join("src")).unwrap();
        fs::write(temp_dir.path().join("src/main.py"), "print('hello')\n").unwrap();

        // Pre-generate .horus/pyproject.toml
        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        let pyproj_path = crate::pyproject_gen::generate(&manifest, temp_dir.path(), false).unwrap();
        let pyproj_content = fs::read_to_string(&pyproj_path).unwrap();
        assert!(pyproj_content.contains("numpy"), "numpy should be in pyproject.toml before removal");
        assert!(pyproj_content.contains("requests"), "requests should be in pyproject.toml before removal");

        // Remove numpy — should regenerate .horus/pyproject.toml without numpy
        run_remove_dep("numpy".to_string()).unwrap();
        std::env::set_current_dir(&original_dir).unwrap();

        let pyproj_after = fs::read_to_string(&pyproj_path).unwrap();
        assert!(!pyproj_after.contains("numpy"), "numpy should be gone from pyproject.toml after removal");
        assert!(pyproj_after.contains("requests"), "requests should remain in pyproject.toml");
    }

    // ── Battle-testing: full add → remove lifecycle via horus.toml ───────

    #[test]
    fn test_add_then_remove_lifecycle() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"lifecycle-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Add a dependency
        run_add("motor-ctrl".to_string(), Some("1.0.0".to_string()), false, false).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("motor-ctrl"));

        // Add a driver
        run_add("lidar".to_string(), Some("rplidar-a2".to_string()), true, false).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.drivers.contains_key("lidar"));

        // Remove the dependency
        run_remove_dep("motor-ctrl".to_string()).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("motor-ctrl"));
        assert!(manifest.drivers.contains_key("lidar"), "driver should survive dep removal");

        // Remove the driver
        run_remove_dep("lidar".to_string()).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.drivers.contains_key("lidar"));

        std::env::set_current_dir(&original_dir).unwrap();
    }

    // ── Battle-testing: run_install + run_remove_dep round-trip ──────────

    #[test]
    fn test_install_then_remove_round_trip() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"round-trip-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Install a crates.io dep
        let _result = run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        );

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("serde"));
        assert!(manifest.dependencies["serde"].is_crates_io());

        // Remove it
        run_remove_dep("serde".to_string()).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("serde"));

        std::env::set_current_dir(&original_dir).unwrap();
    }

    // ── Battle-testing: cargo_gen integration with horus.toml ────────────

    #[test]
    fn test_cargo_gen_from_horus_toml_deps() {
        use crate::manifest::*;
        use std::collections::BTreeMap;

        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("main.rs"), "fn main() {}").unwrap();

        let mut deps = BTreeMap::new();
        deps.insert(
            "serde".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some("1.0".to_string()),
                source: Some(DepSource::CratesIo),
                features: vec!["derive".to_string()],
                optional: false,
                path: None,
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: None,
                cmake_package: None,
                lang: None,
            }),
        );
        deps.insert(
            "numpy".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some(">=1.24".to_string()),
                source: Some(DepSource::PyPI),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: None,
                cmake_package: None,
                lang: None,
            }),
        );

        let manifest = HorusManifest {
            package: PackageInfo {
                name: "test-bot".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
            },
            dependencies: deps,
            dev_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
        };

        // Save to horus.toml then generate Cargo.toml from it
        let horus_path = dir.path().join(HORUS_TOML);
        manifest.save_to(&horus_path).unwrap();

        let cargo_path = crate::cargo_gen::generate(&manifest, dir.path(), &[], false).unwrap();
        let cargo_content = fs::read_to_string(&cargo_path).unwrap();

        // Rust dep should appear in Cargo.toml
        assert!(cargo_content.contains("serde"), "Rust dep should be in generated Cargo.toml");
        assert!(cargo_content.contains("derive"), "features should be preserved");
        // PyPI dep should NOT appear in Cargo.toml
        assert!(!cargo_content.contains("numpy"), "Python dep should not be in Cargo.toml");
    }

    #[test]
    fn test_pyproject_gen_from_horus_toml_deps() {
        use crate::manifest::*;
        use std::collections::BTreeMap;

        let dir = tempfile::tempdir().unwrap();

        let mut deps = BTreeMap::new();
        deps.insert(
            "numpy".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some(">=1.24".to_string()),
                source: Some(DepSource::PyPI),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: None,
                cmake_package: None,
                lang: None,
            }),
        );
        deps.insert(
            "serde".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some("1.0".to_string()),
                source: Some(DepSource::CratesIo),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                branch: None,
                tag: None,
                rev: None,
                apt: None,
                cmake_package: None,
                lang: None,
            }),
        );

        let manifest = HorusManifest {
            package: PackageInfo {
                name: "test-bot".to_string(),
                version: "0.1.0".to_string(),
                description: Some("A test bot".to_string()),
                authors: vec!["Test".to_string()],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
            },
            dependencies: deps,
            dev_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
        };

        let horus_path = dir.path().join(HORUS_TOML);
        manifest.save_to(&horus_path).unwrap();

        let pyproject_path = crate::pyproject_gen::generate(&manifest, dir.path(), false).unwrap();
        let content = fs::read_to_string(&pyproject_path).unwrap();

        // PyPI dep should appear
        assert!(content.contains("numpy>=1.24"), "Python dep should be in pyproject.toml");
        // Rust dep should NOT appear
        assert!(!content.contains("serde"), "Rust dep should not be in pyproject.toml");
    }

    // ── Battle-testing: mixed deps in horus.toml ─────────────────────────

    #[test]
    fn test_horus_toml_mixed_deps_roundtrip() {
        use crate::manifest::*;

        let toml_str = r#"
enable = ["cuda"]

[package]
name = "mixed-robot"
version = "0.1.0"

[dependencies]
horus_library = "0.1.9"
serde = { version = "1.0", features = ["derive"], source = "crates.io" }
numpy = { version = ">=1.24", source = "pypi" }
motor-ctrl = { path = "../motor-ctrl" }
my-fork = { git = "https://github.com/org/repo", branch = "main" }

[drivers]
camera = "opencv"
lidar = "rplidar-a2"
"#;

        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();

        // Verify all dep types parsed correctly
        assert_eq!(manifest.dependencies.len(), 5);
        assert!(manifest.dependencies["horus_library"].is_registry());
        assert!(manifest.dependencies["serde"].is_crates_io());
        assert!(manifest.dependencies["numpy"].is_pypi());
        assert!(manifest.dependencies["motor-ctrl"].is_path());
        assert_eq!(
            manifest.dependencies["my-fork"].effective_source(),
            DepSource::Git
        );

        // Verify language detection from deps
        let langs = manifest.languages_from_deps();
        assert!(langs.contains(&Language::Rust));
        assert!(langs.contains(&Language::Python));

        // Round-trip through save/load
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);
        manifest.save_to(&path).unwrap();
        let loaded = HorusManifest::load_from(&path).unwrap();

        assert_eq!(loaded.dependencies.len(), 5);
        assert_eq!(loaded.drivers.len(), 2);
        assert_eq!(loaded.enable, vec!["cuda"]);
    }

    // ── Battle-testing: drivers config propagates to cargo_gen ────────────

    #[test]
    fn test_drivers_enable_features_in_cargo_gen() {
        use crate::manifest::*;
        use std::collections::BTreeMap;

        let dir = tempfile::tempdir().unwrap();
        fs::write(dir.path().join("main.rs"), "fn main() {}").unwrap();

        let mut drivers = BTreeMap::new();
        drivers.insert("camera".to_string(), DriverValue::Backend("opencv".to_string()));
        drivers.insert("lidar".to_string(), DriverValue::Backend("rplidar-a2".to_string()));

        let manifest = HorusManifest {
            package: PackageInfo {
                name: "hw-bot".to_string(),
                version: "0.1.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "1".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
            },
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            drivers,
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: Default::default(),
        };

        // Save and verify drivers section is preserved
        let horus_path = dir.path().join(HORUS_TOML);
        manifest.save_to(&horus_path).unwrap();
        let loaded = HorusManifest::load_from(&horus_path).unwrap();
        assert_eq!(loaded.drivers.len(), 2);
        assert!(matches!(loaded.drivers["camera"], DriverValue::Backend(ref s) if s == "opencv"));
    }

    // ══════════════════════════════════════════════════════════════════════
    // Phase 3 battle tests: install/remove/search comprehensive coverage
    // ══════════════════════════════════════════════════════════════════════

    // ── run_install: system source writes to manifest ────────────────────

    #[test]
    fn test_run_install_system_source_writes_manifest() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"sys-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "libopencv-dev".to_string(),
            Some("4.5".to_string()),
            false,
            None,
            Some("system".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok(), "system install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("libopencv-dev"));
        assert_eq!(
            manifest.dependencies["libopencv-dev"].effective_source(),
            crate::manifest::DepSource::System
        );
        assert_eq!(manifest.dependencies["libopencv-dev"].version(), Some("4.5"));
    }

    #[test]
    fn test_run_install_system_source_no_version() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"sys-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "curl".to_string(),
            None,
            false,
            None,
            Some("apt".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("curl"));
        assert_eq!(manifest.dependencies["curl"].version(), Some("*"));
    }

    // ── run_install: dev flag with various sources ───────────────────────

    #[test]
    fn test_run_install_dev_crates_io() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"dev-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "criterion".to_string(),
            Some("0.5".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            true, // dev
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok(), "dev crates.io install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dev_dependencies.contains_key("criterion"),
            "criterion should be in dev-dependencies"
        );
        assert!(
            !manifest.dependencies.contains_key("criterion"),
            "criterion should NOT be in regular dependencies"
        );
        assert!(manifest.dev_dependencies["criterion"].is_crates_io());
    }

    #[test]
    fn test_run_install_dev_pypi() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"py-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "pytest".to_string(),
            Some(">=7.0".to_string()),
            false,
            None,
            Some("pypi".to_string()),
            None,
            true,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dev_dependencies.contains_key("pytest"));
        assert!(manifest.dev_dependencies["pytest"].is_pypi());
        assert_eq!(manifest.dev_dependencies["pytest"].version(), Some(">=7.0"));
    }

    #[test]
    fn test_run_install_dev_system_source() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"sys-test-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "valgrind".to_string(),
            None,
            false,
            None,
            Some("system".to_string()),
            None,
            true,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dev_dependencies.contains_key("valgrind"));
        assert!(!manifest.dependencies.contains_key("valgrind"));
    }

    // ── run_install: features flag ──────────────────────────────────────

    #[test]
    fn test_run_install_path_with_features() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"feat-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let local_pkg = temp_dir.path().join("sensor-lib");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"sensor-lib\"\nversion = \"0.3.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "./sensor-lib".to_string(),
            None,
            false,
            None,
            None,
            Some(vec!["lidar".to_string(), "imu".to_string()]),
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok(), "path+features install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("sensor-lib"));
        let feats = manifest.dependencies["sensor-lib"].features();
        assert_eq!(feats.len(), 2);
        assert!(feats.contains(&"lidar".to_string()));
        assert!(feats.contains(&"imu".to_string()));
    }

    #[test]
    fn test_run_install_registry_with_features() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"feat-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _result = run_install(
            "horus-nav".to_string(),
            Some("2.0".to_string()),
            false,
            None,
            Some("registry".to_string()),
            Some(vec!["slam".to_string(), "path-planning".to_string()]),
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        // Registry install writes manifest first, then tries physical install which may fail
        // but manifest should be written

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("horus-nav"));
        let feats = manifest.dependencies["horus-nav"].features();
        assert_eq!(feats.len(), 2);
        assert!(feats.contains(&"slam".to_string()));
    }

    #[test]
    fn test_run_install_pypi_with_features() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"ml-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "torch".to_string(),
            Some("2.0".to_string()),
            false,
            None,
            Some("pip".to_string()),
            Some(vec!["cuda".to_string()]),
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("torch"));
        assert!(manifest.dependencies["torch"].is_pypi());
        let feats = manifest.dependencies["torch"].features();
        assert!(feats.contains(&"cuda".to_string()));
    }

    #[test]
    fn test_run_install_dev_with_features_combined() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"combo-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "tokio".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["full".to_string(), "test-util".to_string()]),
            true, // dev
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dev_dependencies.contains_key("tokio"));
        assert!(!manifest.dependencies.contains_key("tokio"));
        let feats = manifest.dev_dependencies["tokio"].features();
        assert_eq!(feats.len(), 2);
        assert!(feats.contains(&"full".to_string()));
        assert!(feats.contains(&"test-util".to_string()));
    }

    // ── run_install: version constraint operators ────────────────────────

    #[test]
    fn test_run_install_version_constraint_caret() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"ver-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "serde".to_string(),
            Some("^1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["serde"].version(), Some("^1.0"));
    }

    #[test]
    fn test_run_install_version_constraint_tilde() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"ver-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "tokio".to_string(),
            Some("~1.25".to_string()),
            false,
            None,
            Some("cargo".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["tokio"].version(), Some("~1.25"));
    }

    #[test]
    fn test_run_install_version_constraint_gte() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"ver-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "numpy".to_string(),
            Some(">=1.24,<2.0".to_string()),
            false,
            None,
            Some("pypi".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["numpy"].version(), Some(">=1.24,<2.0"));
    }

    #[test]
    fn test_run_install_version_exact_pinned() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"pin-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "rapier3d".to_string(),
            Some("=0.22.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["rapier3d"].version(), Some("=0.22.0"));
    }

    // ── Multiple sequential installs ────────────────────────────────────

    #[test]
    fn test_sequential_installs_accumulate_deps() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"multi-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Install 1: crates.io dep
        run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        )
        .unwrap();

        // Install 2: pypi dep
        run_install(
            "numpy".to_string(),
            Some(">=1.24".to_string()),
            false,
            None,
            Some("pypi".to_string()),
            None,
            false,
        )
        .unwrap();

        // Install 3: system dep
        run_install(
            "libssl-dev".to_string(),
            None,
            false,
            None,
            Some("system".to_string()),
            None,
            false,
        )
        .unwrap();

        // Install 4: dev dep
        run_install(
            "criterion".to_string(),
            Some("0.5".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            true,
        )
        .unwrap();

        // Install 5: path dep
        let local_pkg = temp_dir.path().join("motor-lib");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"motor-lib\"\nversion = \"0.2.0\"\n",
        )
        .unwrap();
        run_install(
            "./motor-lib".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        )
        .unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        // Verify all deps are present in the manifest
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();

        assert_eq!(manifest.dependencies.len(), 4, "should have 4 regular deps");
        assert!(manifest.dependencies.contains_key("serde"));
        assert!(manifest.dependencies.contains_key("numpy"));
        assert!(manifest.dependencies.contains_key("libssl-dev"));
        assert!(manifest.dependencies.contains_key("motor-lib"));

        assert_eq!(manifest.dev_dependencies.len(), 1, "should have 1 dev dep");
        assert!(manifest.dev_dependencies.contains_key("criterion"));

        // Verify sources
        assert!(manifest.dependencies["serde"].is_crates_io());
        assert!(manifest.dependencies["numpy"].is_pypi());
        assert!(manifest.dependencies["motor-lib"].is_path());
    }

    // ── Remove all deps leaving empty sections ──────────────────────────

    #[test]
    fn test_remove_all_deps_leaves_empty_sections() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"strip-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\nalpha = \"1.0\"\nbeta = \"2.0\"\n\n[drivers]\ncamera = \"opencv\"\n",
        )
        .unwrap();

        // Remove everything
        run_remove_dep("alpha".to_string()).unwrap();
        run_remove_dep("beta".to_string()).unwrap();
        run_remove_dep("camera".to_string()).unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.is_empty(), "all deps should be removed");
        assert!(manifest.drivers.is_empty(), "all drivers should be removed");
        // Package info should survive
        assert_eq!(manifest.package.name, "strip-bot");
        assert_eq!(manifest.package.version, "0.1.0");
    }

    // ── Install same package with different version (upgrade) ───────────

    #[test]
    fn test_install_upgrade_version_crates_io() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"upgrade-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Install v1
        run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        )
        .unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["serde"].version(), Some("1.0"));

        // Upgrade to v2
        run_install(
            "serde".to_string(),
            Some("2.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        )
        .unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["serde"].version(), Some("2.0"));
        assert_eq!(manifest.dependencies.len(), 1, "should not duplicate");
    }

    #[test]
    fn test_install_upgrade_registry_to_crates_io() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"switch-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\nmy-lib = \"1.0\"\n",
        )
        .unwrap();

        // Overwrite with crates.io source
        run_install(
            "my-lib".to_string(),
            Some("2.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        )
        .unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies["my-lib"].is_crates_io());
        assert_eq!(manifest.dependencies["my-lib"].version(), Some("2.0"));
    }

    // ── run_remove_dep: dep in multiple sections simultaneously ─────────

    #[test]
    fn test_remove_dep_present_in_deps_and_drivers() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Same name in both [dependencies] and [drivers]
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"dual-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\ncamera = \"1.0\"\n\n[drivers]\ncamera = \"opencv\"\n",
        )
        .unwrap();

        let result = run_remove_dep("camera".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("camera"), "removed from deps");
        assert!(!manifest.drivers.contains_key("camera"), "removed from drivers");
    }

    #[test]
    fn test_remove_dep_present_in_deps_and_dev_deps() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Same name in both [dependencies] and [dev-dependencies]
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"dual-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\ntokio = \"1.0\"\n\n[dev-dependencies]\ntokio = { version = \"1.0\", source = \"crates.io\", features = [\"test-util\"] }\n",
        )
        .unwrap();

        let result = run_remove_dep("tokio".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("tokio"));
        assert!(!manifest.dev_dependencies.contains_key("tokio"));
    }

    // ── Git dep with source flag ────────────────────────────────────────

    #[test]
    fn test_run_install_git_with_source_flag() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"git-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "https://github.com/org/robot-lib.git".to_string(),
            None,
            false,
            None,
            Some("git".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok(), "git install failed: {:?}", result.err());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("robot-lib"));
        assert_eq!(
            manifest.dependencies["robot-lib"].effective_source(),
            crate::manifest::DepSource::Git
        );
    }

    #[test]
    fn test_run_install_git_dep_name_extraction_edge_cases() {
        // Verify name extraction from various git URL formats
        let cases = vec![
            ("https://github.com/user/repo.git", "repo"),
            ("https://github.com/user/my-long-name.git", "my-long-name"),
            ("git://example.com/project", "project"),
            ("https://gitlab.com/group/sub/deep-repo.git", "deep-repo"),
        ];

        for (url, expected) in cases {
            let dep_name = url
                .rsplit('/')
                .next()
                .unwrap_or(url)
                .trim_end_matches(".git")
                .to_string();
            assert_eq!(dep_name, expected, "for URL: {}", url);
        }
    }

    #[test]
    fn test_run_install_git_with_features() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"git-feat-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "https://github.com/org/sensor-fusion.git".to_string(),
            None,
            false,
            None,
            Some("git".to_string()),
            Some(vec!["kalman".to_string(), "ekf".to_string()]),
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("sensor-fusion"));
        let feats = manifest.dependencies["sensor-fusion"].features();
        assert_eq!(feats.len(), 2);
        assert!(feats.contains(&"kalman".to_string()));
        assert!(feats.contains(&"ekf".to_string()));
    }

    // ── Git dep branch/tag/rev via DetailedDependency round-trip ────────

    #[test]
    fn test_detailed_git_dep_with_branch_roundtrip() {
        use crate::manifest::*;

        let toml_str = r#"
[package]
name = "git-branch-bot"
version = "0.1.0"

[dependencies]
my-fork = { git = "https://github.com/org/repo", branch = "develop" }
"#;

        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        assert!(manifest.dependencies.contains_key("my-fork"));

        if let DependencyValue::Detailed(ref d) = manifest.dependencies["my-fork"] {
            assert_eq!(d.git.as_deref(), Some("https://github.com/org/repo"));
            assert_eq!(d.branch.as_deref(), Some("develop"));
            assert!(d.tag.is_none());
            assert!(d.rev.is_none());
        } else {
            panic!("Expected Detailed variant");
        }

        // Round-trip
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);
        manifest.save_to(&path).unwrap();
        let loaded = HorusManifest::load_from(&path).unwrap();
        if let DependencyValue::Detailed(ref d) = loaded.dependencies["my-fork"] {
            assert_eq!(d.branch.as_deref(), Some("develop"));
        }
    }

    #[test]
    fn test_detailed_git_dep_with_tag_roundtrip() {
        use crate::manifest::*;

        let toml_str = r#"
[package]
name = "git-tag-bot"
version = "0.1.0"

[dependencies]
pinned-lib = { git = "https://github.com/org/lib", tag = "v3.2.1" }
"#;

        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        if let DependencyValue::Detailed(ref d) = manifest.dependencies["pinned-lib"] {
            assert_eq!(d.tag.as_deref(), Some("v3.2.1"));
            assert!(d.branch.is_none());
            assert!(d.rev.is_none());
        } else {
            panic!("Expected Detailed variant");
        }
    }

    #[test]
    fn test_detailed_git_dep_with_rev_roundtrip() {
        use crate::manifest::*;

        let toml_str = r#"
[package]
name = "git-rev-bot"
version = "0.1.0"

[dependencies]
exact-lib = { git = "https://github.com/org/exact", rev = "abc123def" }
"#;

        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        if let DependencyValue::Detailed(ref d) = manifest.dependencies["exact-lib"] {
            assert_eq!(d.rev.as_deref(), Some("abc123def"));
            assert!(d.branch.is_none());
            assert!(d.tag.is_none());
        } else {
            panic!("Expected Detailed variant");
        }
    }

    // ── Sequential add + install interop ────────────────────────────────

    #[test]
    fn test_add_then_install_different_deps() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"interop-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Use run_add for one dep
        run_add("motor-ctrl".to_string(), Some("1.0".to_string()), false, false).unwrap();

        // Use run_install for another
        run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        )
        .unwrap();

        // Add a driver via run_add
        run_add("gps".to_string(), Some("ublox".to_string()), true, false).unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 2);
        assert!(manifest.dependencies.contains_key("motor-ctrl"));
        assert!(manifest.dependencies.contains_key("serde"));
        assert!(manifest.drivers.contains_key("gps"));
    }

    // ── Install overwrites add (same dep name) ─────────────────────────

    #[test]
    fn test_install_overwrites_add_same_dep() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"overwrite-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Add dep via run_add — smart resolver detects serde as crates.io
        run_add("serde".to_string(), Some("1.0".to_string()), false, false).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies["serde"].is_crates_io(),
            "smart resolver should detect serde as crates.io");

        // Overwrite with crates.io source via run_install
        run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        )
        .unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 1, "should not duplicate");
        assert!(manifest.dependencies["serde"].is_crates_io());
        assert!(manifest.dependencies["serde"].features().contains(&"derive".to_string()));
    }

    // ── Stress test: many deps ──────────────────────────────────────────

    #[test]
    fn test_many_deps_stress() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"stress-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Add 20 deps via run_add
        for i in 0..20 {
            run_add(
                format!("dep-{}", i),
                Some(format!("{}.0.0", i + 1)),
                false,
                false,
            )
            .unwrap();
        }

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 20);

        for i in 0..20 {
            let name = format!("dep-{}", i);
            assert!(manifest.dependencies.contains_key(&name), "missing {}", name);
            assert_eq!(
                manifest.dependencies[&name].version(),
                Some(format!("{}.0.0", i + 1).as_str())
            );
        }
    }

    // ── Remove in reverse order ─────────────────────────────────────────

    #[test]
    fn test_sequential_remove_preserves_remaining() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"seq-rm-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\nalpha = \"1.0\"\nbeta = \"2.0\"\ngamma = \"3.0\"\ndelta = \"4.0\"\n",
        )
        .unwrap();

        // Remove one by one and verify remaining
        run_remove_dep("beta".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 3);
        assert!(!manifest.dependencies.contains_key("beta"));
        assert!(manifest.dependencies.contains_key("alpha"));
        assert!(manifest.dependencies.contains_key("gamma"));
        assert!(manifest.dependencies.contains_key("delta"));

        run_remove_dep("delta".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 2);

        run_remove_dep("alpha".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 1);
        assert!(manifest.dependencies.contains_key("gamma"));

        run_remove_dep("gamma".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.is_empty());

        std::env::set_current_dir(&original_dir).unwrap();
    }

    // ── run_install: auto-detect without source flag ────────────────────

    #[test]
    fn test_run_install_auto_detects_registry_for_simple_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"auto-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Simple name without source flag => registry
        let _result = run_install(
            "rplidar".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            None, // no source flag
            None,
            false,
        );
        // Registry install may fail at physical install, but manifest should be written

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("rplidar"));
        assert!(manifest.dependencies["rplidar"].is_registry());

        std::env::set_current_dir(&original_dir).unwrap();
    }

    #[test]
    fn test_run_install_auto_detects_path_for_dotslash() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"auto-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let local_pkg = temp_dir.path().join("my-lib");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"my-lib\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "./my-lib".to_string(),
            None,
            false,
            None,
            None, // auto-detect path
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies["my-lib"].is_path());
    }

    #[test]
    fn test_run_install_auto_detects_path_for_tilde() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // No horus.toml => will fail, but we just test detection
        let result = run_install(
            "~/my-pkg".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_err());
        // Error should be about horus.toml not about source detection
        let err_msg = format!("{}", result.unwrap_err());
        assert!(err_msg.contains("horus.toml"), "got: {}", err_msg);
    }

    // ── Detailed dep: optional field ────────────────────────────────────

    #[test]
    fn test_detailed_dep_optional_field_roundtrip() {
        use crate::manifest::*;

        let toml_str = r#"
[package]
name = "opt-bot"
version = "0.1.0"

[dependencies]
cuda-support = { version = "1.0", source = "crates.io", optional = true }
"#;

        let manifest: HorusManifest = toml::from_str(toml_str).unwrap();
        if let DependencyValue::Detailed(ref d) = manifest.dependencies["cuda-support"] {
            assert!(d.optional);
        } else {
            panic!("Expected Detailed variant");
        }

        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join(HORUS_TOML);
        manifest.save_to(&path).unwrap();
        let loaded = HorusManifest::load_from(&path).unwrap();
        if let DependencyValue::Detailed(ref d) = loaded.dependencies["cuda-support"] {
            assert!(d.optional);
        }
    }

    // ── Mixed removal: dep + dev-dep + driver all at once ───────────────

    #[test]
    fn test_remove_dep_from_all_three_sections() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Extreme edge case: same name in all three sections
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"triple-bot\"\nversion = \"0.1.0\"\n\n[dependencies]\ncamera = \"1.0\"\n\n[dev-dependencies]\ncamera = { version = \"1.0\", source = \"crates.io\" }\n\n[drivers]\ncamera = \"opencv\"\n",
        )
        .unwrap();

        let result = run_remove_dep("camera".to_string());
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("camera"));
        assert!(!manifest.dev_dependencies.contains_key("camera"));
        assert!(!manifest.drivers.contains_key("camera"));
    }

    // ── Install + remove + reinstall cycle ──────────────────────────────

    #[test]
    fn test_install_remove_reinstall_cycle() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"cycle-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Install
        run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        )
        .unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("serde"));

        // Remove
        run_remove_dep("serde".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("serde"));

        // Reinstall with different config
        run_install(
            "serde".to_string(),
            Some("2.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string(), "rc".to_string()]),
            false,
        )
        .unwrap();

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("serde"));
        assert_eq!(manifest.dependencies["serde"].version(), Some("2.0"));
        let feats = manifest.dependencies["serde"].features();
        assert_eq!(feats.len(), 2);
        assert!(feats.contains(&"derive".to_string()));
        assert!(feats.contains(&"rc".to_string()));
    }

    // ── run_install: dev git dep ────────────────────────────────────────

    #[test]
    fn test_run_install_dev_git_dep() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"dev-git-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "https://github.com/test/mock-hw.git".to_string(),
            None,
            false,
            None,
            Some("git".to_string()),
            None,
            true, // dev
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dev_dependencies.contains_key("mock-hw"));
        assert!(!manifest.dependencies.contains_key("mock-hw"));
    }

    // ── Empty and special character package names ────────────────────────

    #[test]
    fn test_run_install_hyphenated_package_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"name-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "my-long-hyphenated-package-name".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("my-long-hyphenated-package-name"));
    }

    #[test]
    fn test_run_install_underscored_package_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"name-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_install(
            "horus_library".to_string(),
            Some("0.1.9".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("horus_library"));
    }

    // ── run_add then run_remove_dep for driver ──────────────────────────

    #[test]
    fn test_add_driver_then_remove_cycle() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"driver-cycle\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Add multiple drivers
        run_add("camera".to_string(), Some("opencv".to_string()), true, false).unwrap();
        run_add("lidar".to_string(), Some("rplidar-a2".to_string()), true, false).unwrap();
        run_add("imu".to_string(), None, true, false).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.drivers.len(), 3);

        // Remove one driver
        run_remove_dep("lidar".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.drivers.len(), 2);
        assert!(!manifest.drivers.contains_key("lidar"));
        assert!(manifest.drivers.contains_key("camera"));
        assert!(manifest.drivers.contains_key("imu"));

        // Remove all drivers
        run_remove_dep("camera".to_string()).unwrap();
        run_remove_dep("imu".to_string()).unwrap();
        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.drivers.is_empty());

        std::env::set_current_dir(&original_dir).unwrap();
    }

    // ── run_install: verify Simple vs Detailed stored form ──────────────

    #[test]
    fn test_run_install_registry_no_features_produces_simple() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"form-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _result = run_install(
            "rplidar".to_string(),
            Some("1.2.0".to_string()),
            false,
            None,
            Some("registry".to_string()),
            None,
            false,
        );

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        // Registry with no features => Simple form
        assert!(matches!(manifest.dependencies["rplidar"], crate::manifest::DependencyValue::Simple(ref v) if v == "1.2.0"));
    }

    #[test]
    fn test_run_install_registry_with_features_produces_detailed() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"form-bot2\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let _result = run_install(
            "rplidar".to_string(),
            Some("1.2.0".to_string()),
            false,
            None,
            Some("registry".to_string()),
            Some(vec!["sdk".to_string()]),
            false,
        );

        std::env::set_current_dir(&original_dir).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        // Registry with features => Detailed form
        assert!(matches!(manifest.dependencies["rplidar"], crate::manifest::DependencyValue::Detailed(_)));
        assert!(manifest.dependencies["rplidar"].features().contains(&"sdk".to_string()));
    }

    // ── remove_from_horus_toml helper: removes from both deps and dev ───

    #[test]
    fn test_remove_from_horus_toml_both_deps_and_dev() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        let manifest_content = r#"[package]
name = "test-proj"
version = "0.1.0"

[dependencies]
serde = "1.0"

[dev-dependencies]
serde = { version = "1.0", source = "crates.io" }
"#;
        fs::write(temp_dir.path().join(HORUS_TOML), manifest_content).unwrap();

        remove_from_horus_toml("serde");

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("serde"));
        assert!(!manifest.dev_dependencies.contains_key("serde"));

        std::env::set_current_dir(original_dir).unwrap();
    }

    // ── run_install: multiple features edge case ────────────────────────

    #[test]
    fn test_run_install_many_features() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"many-feat-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let features = vec![
            "derive".to_string(),
            "alloc".to_string(),
            "rc".to_string(),
            "unstable".to_string(),
        ];

        let result = run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(features.clone()),
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        let stored_feats = manifest.dependencies["serde"].features();
        assert_eq!(stored_feats.len(), 4);
        for f in &features {
            assert!(stored_feats.contains(f), "missing feature: {}", f);
        }
    }

    // ── run_install: empty features vec behaves like None ───────────────

    #[test]
    fn test_run_install_empty_features_vec() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"empty-feat-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        // Empty features vec, no version => should produce simple form for crates.io
        let result = run_install(
            "log".to_string(),
            None,
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec![]),
            false,
        );
        std::env::set_current_dir(&original_dir).unwrap();
        assert!(result.is_ok());

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.contains_key("log"));
        assert_eq!(manifest.dependencies["log"].version(), Some("*"));
    }

    // ── Full ecosystem lifecycle: build up, modify, tear down ───────────

    #[test]
    fn test_full_ecosystem_lifecycle() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"full-lifecycle-bot\"\nversion = \"1.0.0\"\n",
        )
        .unwrap();

        // Phase 1: Add deps via different methods
        run_add("motor-ctrl".to_string(), Some("1.0".to_string()), false, false).unwrap();
        run_add("camera".to_string(), Some("opencv".to_string()), true, false).unwrap();
        run_install(
            "serde".to_string(),
            Some("1.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string()]),
            false,
        )
        .unwrap();

        let local_pkg = temp_dir.path().join("sensor-lib");
        fs::create_dir_all(&local_pkg).unwrap();
        fs::write(
            local_pkg.join(HORUS_TOML),
            "[package]\nname = \"sensor-lib\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        run_install(
            "./sensor-lib".to_string(),
            None,
            false,
            None,
            None,
            None,
            false,
        )
        .unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 3);
        assert_eq!(manifest.drivers.len(), 1);

        // Phase 2: Upgrade serde
        run_install(
            "serde".to_string(),
            Some("2.0".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            Some(vec!["derive".to_string(), "rc".to_string()]),
            false,
        )
        .unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies["serde"].version(), Some("2.0"));
        assert_eq!(manifest.dependencies.len(), 3, "count should not change on upgrade");

        // Phase 3: Remove some deps
        run_remove_dep("motor-ctrl".to_string()).unwrap();
        run_remove_dep("camera".to_string()).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 2);
        assert!(manifest.drivers.is_empty());

        // Phase 4: Add dev dep
        run_install(
            "criterion".to_string(),
            Some("0.5".to_string()),
            false,
            None,
            Some("crates.io".to_string()),
            None,
            true,
        )
        .unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dev_dependencies.len(), 1);

        // Phase 5: Remove everything
        run_remove_dep("serde".to_string()).unwrap();
        run_remove_dep("sensor-lib".to_string()).unwrap();
        run_remove_dep("criterion".to_string()).unwrap();

        let manifest =
            HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies.is_empty());
        assert!(manifest.dev_dependencies.is_empty());
        assert!(manifest.drivers.is_empty());
        assert_eq!(manifest.package.name, "full-lifecycle-bot");

        std::env::set_current_dir(&original_dir).unwrap();
    }

    // ── Phase 6: E2E tests — naive user workflows ───────────────────────

    /// E2E: Naive Rust user — horus add → auto-detect → build file → remove → clean
    #[test]
    fn test_e2e_naive_rust_project() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Step 1: Simulate `horus new` — create minimal Rust project
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"my-robot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(temp_dir.path().join("src")).unwrap();
        fs::write(temp_dir.path().join("src/main.rs"), "fn main() {}\n").unwrap();

        // Step 2: `horus add serde` — user doesn't specify source, system auto-detects
        run_add("serde".to_string(), Some("1.0".to_string()), false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dependencies["serde"].is_crates_io(),
            "serde should be auto-detected as crates.io"
        );

        // Step 3: `horus add tokio` — another well-known crate
        run_add("tokio".to_string(), None, false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dependencies["tokio"].is_crates_io(),
            "tokio should be auto-detected as crates.io"
        );

        // Step 4: `horus add xyzzy-robot-widget-99` — truly non-existent package.
        // Not in well-known lists, not on crates.io/PyPI. In Rust-only project,
        // static fallback guesses crates.io (Low confidence), network check finds
        // nothing, so the static guess is kept.
        run_add("xyzzy-robot-widget-99".to_string(), Some("0.1.0".to_string()), false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dependencies["xyzzy-robot-widget-99"].is_crates_io(),
            "unknown dep in Rust-only project should be guessed as crates.io"
        );
        assert_eq!(manifest.dependencies.len(), 3);

        // Step 5: Verify .horus/Cargo.toml is generated with correct sources
        let cargo_path = crate::cargo_gen::generate(&manifest, temp_dir.path(), &[], false).unwrap();
        let cargo_content = fs::read_to_string(&cargo_path).unwrap();
        assert!(cargo_content.contains("serde"), "serde should be in Cargo.toml");
        assert!(cargo_content.contains("tokio"), "tokio should be in Cargo.toml");

        // Step 6: `horus remove serde` — should update horus.toml and regenerate build files
        run_remove_dep("serde".to_string()).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("serde"), "serde removed from manifest");
        assert_eq!(manifest.dependencies.len(), 2);

        // Verify .horus/Cargo.toml was regenerated without serde
        let cargo_after = fs::read_to_string(&cargo_path).unwrap();
        assert!(!cargo_after.contains("serde"), "serde gone from generated Cargo.toml");
        assert!(cargo_after.contains("tokio"), "tokio still in generated Cargo.toml");

        std::env::set_current_dir(&original_dir).unwrap();
    }

    /// E2E: Naive Python user — horus add → auto-detect → build file → remove → clean
    #[test]
    fn test_e2e_naive_python_project() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Step 1: Simulate `horus new --python` — create minimal Python project
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"my-vision-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(temp_dir.path().join("src")).unwrap();
        fs::write(temp_dir.path().join("src/main.py"), "print('hello')\n").unwrap();

        // Step 2: `horus add numpy` — user doesn't know about pip, system auto-detects
        run_add("numpy".to_string(), Some(">=1.24".to_string()), false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dependencies["numpy"].is_pypi(),
            "numpy should be auto-detected as PyPI"
        );

        // Step 3: `horus add requests` — another well-known PyPI package
        run_add("requests".to_string(), None, false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dependencies["requests"].is_pypi(),
            "requests should be auto-detected as PyPI"
        );

        // Step 4: `horus add opencv-python` — PyPI with hyphen
        run_add("opencv-python".to_string(), None, false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(
            manifest.dependencies["opencv-python"].is_pypi(),
            "opencv-python should be auto-detected as PyPI"
        );
        assert_eq!(manifest.dependencies.len(), 3);

        // Step 5: Verify .horus/pyproject.toml is generated correctly
        let pyproj_path =
            crate::pyproject_gen::generate(&manifest, temp_dir.path(), false).unwrap();
        let pyproj_content = fs::read_to_string(&pyproj_path).unwrap();
        assert!(pyproj_content.contains("numpy"), "numpy in pyproject.toml");
        assert!(pyproj_content.contains("requests"), "requests in pyproject.toml");

        // Step 6: `horus remove numpy` — regenerate build files
        run_remove_dep("numpy".to_string()).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(!manifest.dependencies.contains_key("numpy"), "numpy removed");
        assert_eq!(manifest.dependencies.len(), 2);

        let pyproj_after = fs::read_to_string(&pyproj_path).unwrap();
        assert!(!pyproj_after.contains("numpy"), "numpy gone from pyproject.toml");
        assert!(pyproj_after.contains("requests"), "requests still in pyproject.toml");

        std::env::set_current_dir(&original_dir).unwrap();
    }

    /// E2E: Mixed Rust+Python project with deps auto-resolved from both ecosystems
    #[test]
    fn test_e2e_mixed_rust_python_project() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let temp_dir = TempDir::new().unwrap();
        let original_dir = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(temp_dir.path()).unwrap();

        // Step 1: Create mixed-language project
        fs::write(
            temp_dir.path().join(HORUS_TOML),
            "[package]\nname = \"hybrid-bot\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        fs::create_dir_all(temp_dir.path().join("src")).unwrap();
        fs::write(temp_dir.path().join("src/main.rs"), "fn main() {}\n").unwrap();
        fs::write(temp_dir.path().join("src/vision.py"), "import cv2\n").unwrap();

        // Step 2: Add Rust deps — should auto-detect as crates.io
        run_add("serde".to_string(), Some("1.0".to_string()), false, false).unwrap();
        run_add("nalgebra".to_string(), None, false, false).unwrap();

        // Step 3: Add Python deps — should auto-detect as PyPI
        run_add("numpy".to_string(), None, false, false).unwrap();
        run_add("torch".to_string(), None, false, false).unwrap();

        // Step 4: Add unknown dep — should default to registry in mixed context
        run_add("horus-nav".to_string(), Some("0.3.0".to_string()), false, false).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert!(manifest.dependencies["serde"].is_crates_io(), "serde → crates.io");
        assert!(manifest.dependencies["nalgebra"].is_crates_io(), "nalgebra → crates.io");
        assert!(manifest.dependencies["numpy"].is_pypi(), "numpy → pypi");
        assert!(manifest.dependencies["torch"].is_pypi(), "torch → pypi");
        assert!(
            manifest.dependencies["horus-nav"].is_registry(),
            "unknown → registry in mixed project"
        );
        assert_eq!(manifest.dependencies.len(), 5);

        // Step 5: Verify both build files are generated correctly
        let cargo_path = crate::cargo_gen::generate(&manifest, temp_dir.path(), &[], false).unwrap();
        let cargo_content = fs::read_to_string(&cargo_path).unwrap();
        assert!(cargo_content.contains("serde"), "serde in Cargo.toml");
        assert!(cargo_content.contains("nalgebra"), "nalgebra in Cargo.toml");
        // PyPI deps should NOT appear in Cargo.toml
        assert!(!cargo_content.contains("numpy"), "numpy NOT in Cargo.toml");
        assert!(!cargo_content.contains("torch"), "torch NOT in Cargo.toml");

        let pyproj_path =
            crate::pyproject_gen::generate(&manifest, temp_dir.path(), false).unwrap();
        let pyproj_content = fs::read_to_string(&pyproj_path).unwrap();
        assert!(pyproj_content.contains("numpy"), "numpy in pyproject.toml");
        assert!(pyproj_content.contains("torch"), "torch in pyproject.toml");
        // Crates.io deps should NOT appear in pyproject.toml
        assert!(!pyproj_content.contains("serde"), "serde NOT in pyproject.toml");
        assert!(!pyproj_content.contains("nalgebra"), "nalgebra NOT in pyproject.toml");

        // Step 6: Remove one from each ecosystem
        run_remove_dep("serde".to_string()).unwrap();
        run_remove_dep("numpy".to_string()).unwrap();

        let manifest = HorusManifest::load_from(&temp_dir.path().join(HORUS_TOML)).unwrap();
        assert_eq!(manifest.dependencies.len(), 3);
        assert!(!manifest.dependencies.contains_key("serde"));
        assert!(!manifest.dependencies.contains_key("numpy"));

        // Verify both build files regenerated correctly
        let cargo_after = fs::read_to_string(&cargo_path).unwrap();
        assert!(!cargo_after.contains("serde"), "serde gone from Cargo.toml");
        assert!(cargo_after.contains("nalgebra"), "nalgebra still in Cargo.toml");

        let pyproj_after = fs::read_to_string(&pyproj_path).unwrap();
        assert!(!pyproj_after.contains("numpy"), "numpy gone from pyproject.toml");
        assert!(pyproj_after.contains("torch"), "torch still in pyproject.toml");

        std::env::set_current_dir(&original_dir).unwrap();
    }

    /// E2E: Error wrapper produces actionable hints for common build failures
    #[test]
    fn test_e2e_error_wrapper_actionable_hints() {
        use crate::error_wrapper::{cargo_error_hint, format_diagnostic, pip_error_hint};

        // ── Cargo: missing crate ─────────────────────────────────────────
        let stderr = "error: no matching package named `nonexistent-robot-crate` found\nlocation searched: crates.io";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect missing crate pattern");
        assert!(diags[0].message.contains("nonexistent-robot-crate"), "hint names the crate");
        assert!(diags[0].hint.contains("horus add"), "hint suggests horus add");

        let formatted = format_diagnostic(&diags[0]);
        assert!(formatted.contains("horus"), "diagnostic has horus prefix");
        assert!(formatted.contains("hint"), "diagnostic has hint label");
        assert!(formatted.contains("cargo"), "diagnostic names the tool");

        // ── Cargo: linker library missing ────────────────────────────────
        let stderr = "error: could not compile\nnote: ld: cannot find -lssl";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect linker error");
        assert!(diags[0].hint.contains("ssl"), "hint names the missing library");
        assert!(diags[0].hint.contains("apt install"), "hint suggests apt install");

        // ── Cargo: OpenSSL missing ───────────────────────────────────────
        let stderr = "Could not find directory of OpenSSL installation\nopenssl headers missing";
        let diags = cargo_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect openssl error");
        assert!(diags[0].hint.contains("libssl-dev"), "suggests libssl-dev");

        // ── Cargo: no hint for unknown error ─────────────────────────────
        let stderr = "error[E0308]: mismatched types";
        assert!(cargo_error_hint(stderr).is_empty(), "no hint for type errors");

        // ── Pip: package not found ───────────────────────────────────────
        let stderr = "ERROR: No matching distribution found for nonexistent-robot-lib";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect pip not-found pattern");
        assert!(diags[0].message.contains("nonexistent-robot-lib"), "hint names the package");

        // ── Pip: externally managed environment ──────────────────────────
        let stderr = "error: externally-managed-environment\nThis environment is externally managed";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect externally-managed");
        assert!(diags[0].hint.contains("venv"), "suggests venv");

        // ── Pip: build wheel failure ─────────────────────────────────────
        let stderr = "ERROR: Failed building wheel for opencv-python-headless";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect build wheel failure");
        assert!(diags[0].message.contains("opencv-python-headless"), "names the failing package");
        assert!(diags[0].hint.contains("build-essential"), "suggests build tools");

        // ── Pip: version conflict ────────────────────────────────────────
        let stderr = "ERROR: ResolutionImpossible: cannot satisfy requirements";
        let diags = pip_error_hint(stderr);
        assert!(!diags.is_empty(), "should detect version conflict");
        assert!(diags[0].hint.contains("conflict"), "mentions conflict");

        // ── Pip: no hint for unknown error ───────────────────────────────
        let stderr = "SyntaxError: invalid syntax";
        assert!(pip_error_hint(stderr).is_empty(), "no hint for syntax errors");
    }

    // ════════════════════════════════════════════════════════════════════
    // Per-Command Tests: search, publish, list, info, uninstall
    // ════════════════════════════════════════════════════════════════════

    // ── Search Command Tests ────────────────────────────────────────────

    #[test]
    fn test_search_empty_query_does_not_panic() {
        // run_search("") should not panic — may return empty or error gracefully
        // We can't test network calls, but we verify no crash
        let _ = run_search("".to_string(), false, false);
    }

    #[test]
    fn test_search_with_query_does_not_panic() {
        let _ = run_search("rplidar".to_string(), false, false);
    }

    #[test]
    fn test_search_json_flag_does_not_panic() {
        let _ = run_search("camera".to_string(), false, true);
    }

    // ── List Command Tests ──────────────────────────────────────────────

    #[test]
    fn test_list_no_packages_does_not_panic() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let _ = run_list(None, false, false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
    }

    #[test]
    fn test_list_global_flag_does_not_panic() {
        let _ = run_list(None, true, false, false);
    }

    #[test]
    fn test_list_json_flag_does_not_panic() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let _ = run_list(None, false, false, true);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
    }

    #[test]
    fn test_list_with_query_filter() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let _ = run_list(Some("camera".to_string()), false, false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
    }

    // ── Uninstall / Remove Tests ────────────────────────────────────────

    #[test]
    fn test_remove_from_horus_toml_nonexistent_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        // Should not panic with no horus.toml
        remove_from_horus_toml("nonexistent-package");
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
    }

    #[test]
    fn test_remove_from_horus_lock_nonexistent_lockfile() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        // Should not panic with no horus.lock
        remove_from_horus_lock("nonexistent-package");
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
    }

    #[test]
    fn test_remove_dep_no_manifest_errors() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_remove_dep("serde".to_string());
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
        // Should fail gracefully without manifest
        assert!(result.is_err());
    }

    #[test]
    fn test_remove_dep_from_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml = r#"
[package]
name = "test"
version = "0.1.0"
edition = "1"

[dependencies]
serde = "1.0"
tokio = "1.0"
"#;
        std::fs::write(tmp.path().join("horus.toml"), toml).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_remove_dep("serde".to_string());
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // Verify serde removed from manifest
        let content = std::fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(!content.contains("serde"));
        assert!(content.contains("tokio")); // other deps preserved
    }

    // ── Publish Tests (helper logic) ────────────────────────────────────

    #[test]
    fn test_publish_no_manifest_errors() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_publish(true); // dry_run = true
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
        // Should fail — no horus.toml
        assert!(result.is_err());
    }

    // ── Add Command Tests ───────────────────────────────────────────────

    #[test]
    fn test_add_dep_no_manifest_errors() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_add("serde".to_string(), Some("1.0".to_string()), false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);
        // Should fail — no horus.toml
        assert!(result.is_err());
    }

    #[test]
    fn test_add_dep_to_manifest() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml = r#"
[package]
name = "test"
version = "0.1.0"
edition = "1"
"#;
        std::fs::write(tmp.path().join("horus.toml"), toml).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_add("serde".to_string(), Some("1.0".to_string()), false, false);
        std::env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        let content = std::fs::read_to_string(tmp.path().join("horus.toml")).unwrap();
        assert!(content.contains("serde"));
    }

    // ── Info Command Tests ──────────────────────────────────────────────

    #[test]
    fn test_print_package_info_nonexistent_package() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::create_dir_all(tmp.path().join("packages")).unwrap();
        let result = print_package_info(
            tmp.path().join("packages").as_path(),
            "nonexistent-pkg",
            tmp.path(),
        );
        assert!(!result, "Should return false for missing package");
    }

    #[test]
    fn test_read_package_name_from_horus_toml_in_dir() {
        let tmp = tempfile::TempDir::new().unwrap();
        std::fs::write(
            tmp.path().join("horus.toml"),
            "[package]\nname = \"my-pkg\"\nversion = \"0.1.0\"\nedition = \"1\"\n",
        )
        .unwrap();
        let name = read_package_name_from_path(tmp.path());
        assert!(name.is_ok());
        assert_eq!(name.unwrap(), "my-pkg");
    }

    #[test]
    fn test_read_package_name_fallback_to_dirname() {
        let tmp = tempfile::TempDir::new().unwrap();
        // No manifest files — should fall back to directory name
        let name = read_package_name_from_path(tmp.path());
        assert!(name.is_ok());
        let n = name.unwrap();
        assert!(!n.is_empty());
    }
}
