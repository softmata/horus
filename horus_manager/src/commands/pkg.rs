//! Package management commands including plugin support
//!
//! This module provides helper functions for pkg subcommands that
//! involve plugin management.

use crate::cargo_utils::is_executable;
use crate::config::{CARGO_TOML, HORUS_YAML};
use crate::plugins::{
    CommandInfo, Compatibility, PluginEntry, PluginRegistry, PluginResolver, PluginScope,
    PluginSource, VerificationStatus, HORUS_VERSION,
};
use anyhow::{anyhow, Result};
use chrono::Utc;
use colored::*;
use serde_json;
use std::fs;
use std::path::{Path, PathBuf};

/// Detect if a package has CLI plugin capabilities
///
/// Returns Some(PluginMetadata) if the package provides CLI extensions
pub fn detect_plugin_metadata(package_dir: &Path) -> Option<PluginMetadata> {
    // Check for horus.yaml with plugin configuration
    let horus_yaml = package_dir.join(HORUS_YAML);
    if horus_yaml.exists() {
        if let Ok(content) = fs::read_to_string(&horus_yaml) {
            if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&content) {
                if let Some(plugin) = yaml.get("plugin") {
                    return parse_plugin_yaml(plugin, &yaml, package_dir);
                }
            }
        }
    }

    // Check for Cargo.toml with [package.metadata.horus] plugin config
    let cargo_toml = package_dir.join(CARGO_TOML);
    if cargo_toml.exists() {
        if let Ok(content) = fs::read_to_string(&cargo_toml) {
            if let Ok(toml) = content.parse::<toml::Table>() {
                if let Some(metadata) = toml
                    .get("package")
                    .and_then(|p| p.get("metadata"))
                    .and_then(|m| m.get("horus"))
                {
                    return parse_plugin_toml(metadata, &toml, package_dir);
                }
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

fn parse_plugin_yaml(
    plugin: &serde_yaml::Value,
    yaml: &serde_yaml::Value,
    package_dir: &Path,
) -> Option<PluginMetadata> {
    let command = plugin.get("command")?.as_str()?.to_string();
    let binary_rel = plugin.get("binary")?.as_str()?;
    let binary = package_dir.join(binary_rel);

    let package_name = yaml
        .get("name")
        .and_then(|n| n.as_str())
        .unwrap_or("unknown")
        .to_string();

    let version = yaml
        .get("version")
        .and_then(|v| v.as_str())
        .unwrap_or("0.0.0")
        .to_string();

    let commands = plugin
        .get("subcommands")
        .and_then(|s| s.as_sequence())
        .map(|seq| {
            seq.iter()
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
                .and_then(|p| p.as_sequence())
                .map(|seq| {
                    seq.iter()
                        .filter_map(|v| v.as_str().map(|s| s.to_string()))
                        .collect()
                })
                .unwrap_or_default(),
        })
        .unwrap_or_default();

    let permissions = plugin
        .get("permissions")
        .and_then(|p| p.as_sequence())
        .map(|seq| {
            seq.iter()
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
    // Try horus.yaml
    let horus_yaml = package_dir.join(HORUS_YAML);
    if horus_yaml.exists() {
        if let Ok(content) = fs::read_to_string(&horus_yaml) {
            if let Ok(yaml) = serde_yaml::from_str::<serde_yaml::Value>(&content) {
                if let Some(version) = yaml.get("version").and_then(|v| v.as_str()) {
                    return Some(version.to_string());
                }
            }
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

    // Create symlink in bin directory
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

    // Create new symlink
    #[cfg(unix)]
    std::os::unix::fs::symlink(&metadata.binary, &symlink_path)?;
    #[cfg(windows)]
    std::os::windows::fs::symlink_file(&metadata.binary, &symlink_path)?;

    // Update plugin registry
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

    println!(
        "   {} Registered CLI plugin: {}",
        "🔌".cyan(),
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
    let mut resolver = PluginResolver::new()?;

    // Remove from registry
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

    // Remove symlink from bin directory
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

    println!(
        "   {} Unregistered plugin: {}",
        "🔌".dimmed(),
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
            println!("{} Plugin '{}' enabled", "✓".green(), command.green());
            return Ok(());
        }
    }

    if resolver.global().is_disabled(command) {
        resolver.global_mut().enable_plugin(command)?;
        resolver.save_global()?;
        println!("{} Plugin '{}' enabled", "✓".green(), command.green());
        return Ok(());
    }

    Err(anyhow!("Plugin '{}' is not disabled", command))
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
                "✓".yellow(),
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
            "✓".yellow(),
            command.yellow()
        );
        return Ok(());
    }

    Err(anyhow!("Plugin '{}' not found", command))
}

/// Verify plugin integrity
pub fn verify_plugins(plugin_name: Option<&str>) -> Result<()> {
    let resolver = PluginResolver::new()?;
    let results = resolver.verify_all();

    if results.is_empty() {
        println!("{} No plugins installed", "ℹ".cyan());
        return Ok(());
    }

    println!("{} Verifying plugins...\n", "🔍".cyan());

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
                    "✓".green(),
                    result.command.green(),
                    scope_str
                );
            }
            VerificationStatus::ChecksumMismatch => {
                all_valid = false;
                println!(
                    "  {} {} {} checksum MISMATCH",
                    "✗".red(),
                    result.command.red(),
                    scope_str
                );
            }
            VerificationStatus::Error => {
                all_valid = false;
                println!(
                    "  {} {} {} error: {}",
                    "✗".red(),
                    result.command.red(),
                    scope_str,
                    result.error.as_deref().unwrap_or("unknown error")
                );
            }
        }
    }

    println!();

    if all_valid {
        println!("{} All plugins verified successfully", "✓".green().bold());
        Ok(())
    } else {
        Err(anyhow!(
            "Some plugins failed verification. Run 'horus pkg install <package>' to reinstall."
        ))
    }
}

/// List installed plugins
pub fn list_plugins(show_global: bool, show_project: bool) -> Result<()> {
    let resolver = PluginResolver::new()?;

    let mut has_output = false;

    // Project plugins
    if show_project {
        if let Some(project) = resolver.project() {
            if !project.plugins.is_empty() {
                has_output = true;
                println!("{} Project plugins:\n", "🔌".cyan());
                for (cmd, entry) in &project.plugins {
                    let status = if entry.binary.exists() {
                        "✓".green()
                    } else {
                        "✗".red()
                    };
                    println!(
                        "  {} {}  {} v{}",
                        status,
                        cmd.green(),
                        entry.package.dimmed(),
                        entry.version.dimmed()
                    );

                    if !entry.commands.is_empty() {
                        for subcmd in &entry.commands {
                            println!(
                                "      {} {} - {}",
                                "•".dimmed(),
                                subcmd.name,
                                subcmd.description.dimmed()
                            );
                        }
                    }
                }
            }

            // Show disabled
            if !project.disabled.is_empty() {
                println!("\n  {} Disabled:", "⊘".dimmed());
                for (cmd, info) in &project.disabled {
                    println!(
                        "    {} {} - {}",
                        cmd.dimmed(),
                        info.plugin.version.dimmed(),
                        info.reason.dimmed()
                    );
                }
            }
        }
    }

    // Global plugins
    if show_global {
        if !resolver.global().plugins.is_empty() {
            if has_output {
                println!();
            }
            has_output = true;
            println!("{} Global plugins:\n", "🔌".cyan());
            for (cmd, entry) in &resolver.global().plugins {
                let status = if entry.binary.exists() {
                    "✓".green()
                } else {
                    "✗".red()
                };
                println!(
                    "  {} {}  {} v{}",
                    status,
                    cmd.green(),
                    entry.package.dimmed(),
                    entry.version.dimmed()
                );

                if !entry.commands.is_empty() {
                    for subcmd in &entry.commands {
                        println!(
                            "      {} {} - {}",
                            "•".dimmed(),
                            subcmd.name,
                            subcmd.description.dimmed()
                        );
                    }
                }
            }
        }

        // Show disabled
        if !resolver.global().disabled.is_empty() {
            println!("\n  {} Disabled:", "⊘".dimmed());
            for (cmd, info) in &resolver.global().disabled {
                println!(
                    "    {} {} - {}",
                    cmd.dimmed(),
                    info.plugin.version.dimmed(),
                    info.reason.dimmed()
                );
            }
        }
    }

    if !has_output {
        println!("{} No plugins installed", "ℹ".cyan());
        println!(
            "\n  Install plugins with: {}",
            "horus pkg install <package>".cyan()
        );
    }

    Ok(())
}

// ── Package management command handlers ──────────────────────────────────

use crate::{registry, workspace, yaml_utils};
use horus_core::error::{HorusError, HorusResult};

/// Install a package from registry or local path
pub fn run_install(
    package: String,
    ver: Option<String>,
    global: bool,
    target: Option<String>,
) -> HorusResult<()> {
    // Check if package is actually a path
    if yaml_utils::is_path_like(&package) {
        // Path dependency installation
        if global {
            return Err(HorusError::Config(
                "Cannot install path dependencies globally. Path dependencies must be local."
                    .to_string(),
            ));
        }

        println!(
            "{} Installing path dependency: {}",
            "▶".cyan(),
            package.green()
        );

        // Resolve path
        let path = PathBuf::from(&package);
        let absolute_path = if path.is_absolute() {
            path.clone()
        } else {
            std::env::current_dir()
                .map_err(|e| HorusError::Config(e.to_string()))?
                .join(&path)
        };

        // Verify path exists and is a directory
        if !absolute_path.exists() {
            return Err(HorusError::Config(format!(
                "Path does not exist: {}",
                absolute_path.display()
            )));
        }
        if !absolute_path.is_dir() {
            return Err(HorusError::Config(format!(
                "Path is not a directory: {}",
                absolute_path.display()
            )));
        }

        // Read package name from the path
        let package_name = yaml_utils::read_package_name_from_path(&absolute_path)
            .map_err(|e| HorusError::Config(e.to_string()))?;

        println!(
            "  {} Detected package name: {}",
            "▸".cyan(),
            package_name.cyan()
        );

        // Determine installation target
        let install_target = if let Some(target_name) = target {
            let registry = workspace::WorkspaceRegistry::load()
                .map_err(|e| HorusError::Config(e.to_string()))?;
            let ws = registry.find_by_name(&target_name).ok_or_else(|| {
                HorusError::Config(format!("Workspace '{}' not found", target_name))
            })?;
            workspace::InstallTarget::Local(ws.path.clone())
        } else {
            workspace::detect_or_select_workspace(true)
                .map_err(|e| HorusError::Config(e.to_string()))?
        };

        // Install using install_from_path
        let client = registry::RegistryClient::new();
        let workspace_path = match &install_target {
            workspace::InstallTarget::Local(p) => p.clone(),
            _ => unreachable!(), // Already blocked global above
        };

        // Pass None for base_dir - CLI paths are resolved relative to current_dir
        client
            .install_from_path(&package_name, &absolute_path, install_target, None)
            .map_err(|e| HorusError::Config(e.to_string()))?;

        // Update horus.yaml with path dependency
        let horus_yaml_path = workspace_path.join(HORUS_YAML);
        if horus_yaml_path.exists() {
            if let Err(e) = yaml_utils::add_path_dependency_to_horus_yaml(
                &horus_yaml_path,
                &package_name,
                &package, // Use original path as provided by user
            ) {
                println!("  {} Failed to update horus.yaml: {}", "⚠".yellow(), e);
            } else {
                println!("  {} Updated horus.yaml", "✓".green());
            }
        }

        println!("{} Path dependency installed successfully!", "✓".green());
        Ok(())
    } else {
        // Registry dependency installation
        let install_target = if global {
            workspace::InstallTarget::Global
        } else if let Some(target_name) = target {
            let registry = workspace::WorkspaceRegistry::load()
                .map_err(|e| HorusError::Config(e.to_string()))?;
            let ws = registry.find_by_name(&target_name).ok_or_else(|| {
                HorusError::Config(format!("Workspace '{}' not found", target_name))
            })?;
            workspace::InstallTarget::Local(ws.path.clone())
        } else {
            workspace::detect_or_select_workspace(true)
                .map_err(|e| HorusError::Config(e.to_string()))?
        };

        let client = registry::RegistryClient::new();
        client
            .install_to_target(&package, ver.as_deref(), install_target.clone())
            .map_err(|e| HorusError::Config(e.to_string()))?;

        // Update horus.yaml if installing locally
        if let workspace::InstallTarget::Local(workspace_path) = install_target {
            let horus_yaml_path = workspace_path.join(HORUS_YAML);
            if horus_yaml_path.exists() {
                let version = ver.as_deref().unwrap_or("latest");
                if let Err(e) =
                    yaml_utils::add_dependency_to_horus_yaml(&horus_yaml_path, &package, version)
                {
                    println!("  {} Failed to update horus.yaml: {}", "⚠".yellow(), e);
                }
            }
        }

        Ok(())
    }
}

/// Remove a package
pub fn run_remove(package: String, global: bool, target: Option<String>) -> HorusResult<()> {
    println!("{} Removing {}...", "▶".cyan(), package.yellow());

    // Track workspace path for horus.yaml update
    let workspace_path = if global {
        None
    } else if let Some(target_name) = &target {
        let reg =
            workspace::WorkspaceRegistry::load().map_err(|e| HorusError::Config(e.to_string()))?;
        let ws = reg
            .find_by_name(target_name)
            .ok_or_else(|| HorusError::Config(format!("Workspace '{}' not found", target_name)))?;
        Some(ws.path.clone())
    } else {
        workspace::find_workspace_root()
    };

    let remove_dir = if global {
        // Remove from global cache
        let global_cache =
            crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

        // Find versioned directory
        let mut found = None;
        if global_cache.exists() {
            for entry in
                fs::read_dir(&global_cache).map_err(|e| HorusError::Config(e.to_string()))?
            {
                let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                let name = entry.file_name().to_string_lossy().to_string();
                if name == package || name.starts_with(&format!("{}@", package)) {
                    found = Some(entry.path());
                    break;
                }
            }
        }
        found.ok_or_else(|| {
            HorusError::Config(format!("Package {} not found in global cache", package))
        })?
    } else if let Some(target_name) = &target {
        // Remove from specific workspace
        let reg =
            workspace::WorkspaceRegistry::load().map_err(|e| HorusError::Config(e.to_string()))?;
        let ws = reg
            .find_by_name(target_name)
            .ok_or_else(|| HorusError::Config(format!("Workspace '{}' not found", target_name)))?;
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
        crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?
    } else if let Some(target_name) = &target {
        let reg =
            workspace::WorkspaceRegistry::load().map_err(|e| HorusError::Config(e.to_string()))?;
        let ws = reg
            .find_by_name(target_name)
            .ok_or_else(|| HorusError::Config(format!("Workspace '{}' not found", target_name)))?;
        ws.path.join(".horus/packages")
    } else if let Some(root) = workspace::find_workspace_root() {
        root.join(".horus/packages")
    } else {
        PathBuf::from(".horus/packages")
    };

    let system_ref = packages_dir.join(format!("{}.system.json", package));
    if system_ref.exists() {
        // Read to determine package type
        let content = fs::read_to_string(&system_ref)
            .map_err(|e| HorusError::Config(format!("Failed to read system reference: {}", e)))?;
        let metadata: serde_json::Value = serde_json::from_str(&content)
            .map_err(|e| HorusError::Config(format!("failed to parse system reference: {}", e)))?;

        // Remove reference file
        fs::remove_file(&system_ref)
            .map_err(|e| HorusError::Config(format!("Failed to remove system reference: {}", e)))?;

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
                        HorusError::Config(format!("Failed to remove binary link: {}", e))
                    })?;
                    println!("  Removed binary link for {}", package);
                }
            }
        }

        println!("  Removed system package reference for {}", package);

        // Update horus.yaml if removing from local workspace
        if let Some(ws_path) = workspace_path {
            let horus_yaml_path = ws_path.join(HORUS_YAML);
            if horus_yaml_path.exists() {
                let content = fs::read_to_string(&horus_yaml_path)
                    .map_err(|e| HorusError::Config(e.to_string()))?;

                // Remove package from dependencies list
                let lines: Vec<&str> = content.lines().collect();
                let mut new_lines = Vec::new();
                let mut in_deps = false;

                for line in lines {
                    if line.trim() == "dependencies:" {
                        in_deps = true;
                        new_lines.push(line);
                    } else if in_deps && line.starts_with("  -") {
                        let dep = line.trim_start_matches("  -").trim();
                        if dep != package && !dep.starts_with(&format!("{}@", package)) {
                            new_lines.push(line);
                        }
                    } else {
                        if in_deps && !line.starts_with("  ") {
                            in_deps = false;
                        }
                        new_lines.push(line);
                    }
                }

                let new_content = new_lines.join("\n") + "\n";
                fs::write(&horus_yaml_path, new_content)
                    .map_err(|e| HorusError::Config(e.to_string()))?;
            }
        }

        return Ok(());
    }

    if !remove_dir.exists() {
        println!("  Package {} is not installed", package);
        return Ok(());
    }

    // Remove package directory
    fs::remove_dir_all(&remove_dir)
        .map_err(|e| HorusError::Config(format!("Failed to remove package: {}", e)))?;

    println!("  Removed {} from {}", package, remove_dir.display());

    // Update horus.yaml if removing from local workspace
    if let Some(ws_path) = workspace_path {
        let horus_yaml_path = ws_path.join(HORUS_YAML);
        if horus_yaml_path.exists() {
            if let Err(e) =
                yaml_utils::remove_dependency_from_horus_yaml(&horus_yaml_path, &package)
            {
                println!("  {} Failed to update horus.yaml: {}", "⚠".yellow(), e);
            }
        }
    }

    Ok(())
}

/// List packages (local, global, or search)
pub fn run_list(query: Option<String>, global: bool, all: bool) -> HorusResult<()> {
    let client = registry::RegistryClient::new();

    if let Some(q) = query {
        // Search registry marketplace
        println!(
            "{} Searching registry marketplace for '{}'...",
            "▶".cyan(),
            q
        );
        let results = client
            .search(&q, None, None)
            .map_err(|e| HorusError::Config(e.to_string()))?;

        if results.is_empty() {
            println!("  No packages found in marketplace matching '{}'", q);
        } else {
            println!(
                "\n{} Found {} package(s) in marketplace:\n",
                "✓".green(),
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
        let global_cache =
            crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

        // Show local packages
        println!("{} Local packages:\n", "📦".cyan());
        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
            root.join(".horus/packages")
        } else {
            PathBuf::from(".horus/packages")
        };

        if packages_dir.exists() {
            let mut has_local = false;
            for entry in
                fs::read_dir(&packages_dir).map_err(|e| HorusError::Config(e.to_string()))?
            {
                let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                let entry_path = entry.path();

                // Skip if it's a metadata file
                if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                    continue;
                }

                if entry
                    .file_type()
                    .map_err(|e| HorusError::Config(e.to_string()))?
                    .is_dir()
                    || entry
                        .file_type()
                        .map_err(|e| HorusError::Config(e.to_string()))?
                        .is_symlink()
                {
                    has_local = true;
                    let name = entry.file_name().to_string_lossy().to_string();
                    if !print_package_info(&packages_dir, &name, &entry_path) {
                        println!("   {}", name.yellow());
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
        println!("\n{} Global cache packages:\n", "🌐".cyan());
        if global_cache.exists() {
            let mut has_global = false;
            for entry in
                fs::read_dir(&global_cache).map_err(|e| HorusError::Config(e.to_string()))?
            {
                let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
                if entry
                    .file_type()
                    .map_err(|e| HorusError::Config(e.to_string()))?
                    .is_dir()
                {
                    has_global = true;
                    let name = entry.file_name().to_string_lossy().to_string();
                    println!("   {}", name.yellow());
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
        println!("{} Global cache packages:\n", "🌐".cyan());
        let global_cache =
            crate::paths::cache_dir().map_err(|e| HorusError::Config(e.to_string()))?;

        if !global_cache.exists() {
            println!("  No global packages yet");
            return Ok(());
        }

        for entry in fs::read_dir(&global_cache).map_err(|e| HorusError::Config(e.to_string()))? {
            let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
            if entry
                .file_type()
                .map_err(|e| HorusError::Config(e.to_string()))?
                .is_dir()
            {
                let name = entry.file_name().to_string_lossy().to_string();
                println!("   {}", name.yellow());
            }
        }
    } else {
        // List local workspace packages (default)
        let packages_dir = if let Some(root) = workspace::find_workspace_root() {
            root.join(".horus/packages")
        } else {
            PathBuf::from(".horus/packages")
        };

        println!("{} Local packages:\n", "📦".cyan());

        if !packages_dir.exists() {
            println!("  No packages installed yet");
            return Ok(());
        }

        for entry in fs::read_dir(&packages_dir).map_err(|e| HorusError::Config(e.to_string()))? {
            let entry = entry.map_err(|e| HorusError::Config(e.to_string()))?;
            let entry_path = entry.path();

            // Skip if it's a metadata file
            if entry_path.extension().and_then(|s| s.to_str()) == Some("json") {
                continue;
            }

            if entry
                .file_type()
                .map_err(|e| HorusError::Config(e.to_string()))?
                .is_dir()
                || entry
                    .file_type()
                    .map_err(|e| HorusError::Config(e.to_string()))?
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
pub fn run_publish(freeze: bool, dry_run: bool) -> HorusResult<()> {
    let client = registry::RegistryClient::new();
    client
        .publish(None, dry_run, None)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    // If --freeze flag is set, also generate freeze file
    if freeze {
        println!("\n{} Generating freeze file...", "▶".cyan());
        let manifest = client
            .freeze()
            .map_err(|e| HorusError::Config(e.to_string()))?;

        let freeze_file = "horus-freeze.yaml";
        let yaml =
            serde_yaml::to_string(&manifest).map_err(|e| HorusError::Config(e.to_string()))?;
        fs::write(freeze_file, yaml).map_err(|e| HorusError::Config(e.to_string()))?;

        println!("  Environment also frozen to {}", freeze_file);
    }

    Ok(())
}

/// Update packages
pub fn run_update(package: Option<String>, global: bool, dry_run: bool) -> HorusResult<()> {
    let client = registry::RegistryClient::new();
    client
        .update_packages(package.as_deref(), global, dry_run)
        .map_err(|e| HorusError::Config(e.to_string()))?;
    Ok(())
}

/// Generate signing keypair
pub fn run_keygen() -> HorusResult<()> {
    registry::generate_signing_keypair().map_err(|e| HorusError::Config(e.to_string()))?;
    Ok(())
}

/// Unpublish a package from the registry
pub fn run_unpublish(package: String, version: String, yes: bool) -> HorusResult<()> {
    use std::io::{self, Write};

    println!(
        "{} Unpublishing {} v{}...",
        "▶".cyan(),
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
        io::stdin()
            .read_line(&mut confirmation)
            .map_err(|e| HorusError::Config(format!("Failed to read input: {}", e)))?;

        if confirmation.trim() != package {
            println!("  Package name mismatch. Unpublish cancelled.");
            return Ok(());
        }
    }

    // Call unpublish API
    let client = registry::RegistryClient::new();
    client
        .unpublish(&package, &version)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    println!(
        "\n  Successfully unpublished {} v{}",
        package.green(),
        version.green()
    );
    println!("   The package is no longer available on the registry");

    Ok(())
}

/// Add a dependency to horus.yaml (does NOT install - deferred to `horus run`)
pub fn run_add(name: String, ver: Option<String>, driver: bool, plugin: bool) -> HorusResult<()> {
    // Find horus.yaml in current directory or workspace
    let workspace_path = workspace::find_workspace_root()
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_default());
    let horus_yaml_path = workspace_path.join(HORUS_YAML);

    if !horus_yaml_path.exists() {
        return Err(HorusError::Config(
            "No horus.yaml found. Run 'horus init' or 'horus new' first.".to_string(),
        ));
    }

    // Determine package type for display (registry already tracks type via package_type metadata)
    let pkg_type = if driver {
        "driver"
    } else if plugin {
        "plugin"
    } else {
        "node"
    };

    let version = ver.as_deref().unwrap_or("latest");

    // All dependencies use the same format: name@version (no type prefix)
    // The registry already has package_type metadata — the CLI doesn't need to track it in horus.yaml
    let dep_string = if version == "latest" {
        name.clone()
    } else {
        format!("{}@{}", name, version)
    };

    // Add to horus.yaml
    match yaml_utils::add_dependency_to_horus_yaml(&horus_yaml_path, &dep_string, version) {
        Ok(_) => {
            println!("{} Added '{}' to horus.yaml", "✓".green(), name.cyan());
            println!("  Type: {}", pkg_type.dimmed());
            if version != "latest" {
                println!("  Version: {}", version.dimmed());
            }
            println!();
            println!("Run {} to install dependencies.", "horus run".cyan().bold());
        }
        Err(e) => {
            return Err(HorusError::Config(format!(
                "Failed to update horus.yaml: {}",
                e
            )));
        }
    }

    Ok(())
}

/// Remove a dependency from horus.yaml (does NOT delete from cache)
pub fn run_remove_dep(name: String) -> HorusResult<()> {
    // Find horus.yaml in current directory or workspace
    let workspace_path = workspace::find_workspace_root()
        .unwrap_or_else(|| std::env::current_dir().unwrap_or_default());
    let horus_yaml_path = workspace_path.join(HORUS_YAML);

    if !horus_yaml_path.exists() {
        return Err(HorusError::Config(
            "No horus.yaml found in current directory or workspace.".to_string(),
        ));
    }

    // Remove from horus.yaml
    match yaml_utils::remove_dependency_from_horus_yaml(&horus_yaml_path, &name) {
        Ok(_) => {
            println!("{} Removed '{}' from horus.yaml", "✓".green(), name.cyan());
            println!();
            println!("Note: Package remains in cache (~/.horus/cache/).");
            println!(
                "Run {} to clean unused packages.",
                "horus cache clean".dimmed()
            );
        }
        Err(e) => {
            // Check if it's a "not found" error
            let err_str = e.to_string();
            if err_str.contains("not found") {
                println!("{} '{}' is not in horus.yaml", "!".yellow(), name);
            } else {
                return Err(HorusError::Config(format!(
                    "Failed to update horus.yaml: {}",
                    e
                )));
            }
        }
    }

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    #[test]
    fn test_detect_plugin_metadata_none() {
        let temp_dir = TempDir::new().unwrap();
        let result = detect_plugin_metadata(temp_dir.path());
        assert!(result.is_none());
    }

    #[test]
    fn test_detect_version_from_horus_yaml() {
        let temp_dir = TempDir::new().unwrap();
        let yaml_path = temp_dir.path().join(HORUS_YAML);
        fs::write(&yaml_path, "name: test\nversion: 1.2.3\n").unwrap();

        let version = detect_version(temp_dir.path());
        assert_eq!(version, Some("1.2.3".to_string()));
    }
}
