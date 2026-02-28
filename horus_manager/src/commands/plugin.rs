//! Plugin command - search, discover, inspect, install, and remove HORUS plugins

use crate::{registry, workspace, yaml_utils};
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::path::PathBuf;

/// Search for plugins by query
pub fn run_search(query: String) -> HorusResult<()> {
    use crate::plugins::PluginDiscovery;

    let mut discovery = PluginDiscovery::new();

    // Add softmata workspace for local plugins
    if let Ok(cwd) = std::env::current_dir() {
        // Check if we're in softmata workspace
        if cwd.join("horus").exists() {
            discovery.add_workspace_path(cwd);
        } else if let Some(parent) = cwd.parent() {
            if parent.join("horus").exists() {
                discovery.add_workspace_path(parent.to_path_buf());
            }
        }
    }

    let results = discovery
        .search(&query)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    if results.is_empty() {
        println!(
            "{}",
            format!("No plugins found matching '{}'", query).yellow()
        );
    } else {
        println!(
            "{}",
            format!("Found {} plugins matching '{}':", results.len(), query)
                .cyan()
                .bold()
        );
        println!();
        for plugin in results {
            let source_badge = match plugin.source {
                crate::plugins::PluginSourceType::Local => "[LOCAL]".yellow(),
                crate::plugins::PluginSourceType::Registry => "[REGISTRY]".green(),
                crate::plugins::PluginSourceType::CratesIo => "[CRATES.IO]".blue(),
                crate::plugins::PluginSourceType::Git => "[GIT]".magenta(),
            };
            println!(
                "  {} {} {} {}",
                plugin.name.white().bold(),
                format!("v{}", plugin.version).dimmed(),
                source_badge,
                if plugin.has_prebuilt {
                    "[prebuilt]".green()
                } else {
                    "".normal()
                }
            );
            println!("    {}", plugin.description.dimmed());
            if !plugin.features.is_empty() {
                println!("    Features: {}", plugin.features.join(", ").cyan());
            }
            println!();
        }
    }
    Ok(())
}

/// List all available plugins grouped by category
pub fn run_available(include_local: bool) -> HorusResult<()> {
    use crate::plugins::PluginDiscovery;

    let mut discovery = PluginDiscovery::new();

    if include_local {
        if let Ok(cwd) = std::env::current_dir() {
            if cwd.join("horus").exists() {
                discovery.add_workspace_path(cwd);
            } else if let Some(parent) = cwd.parent() {
                if parent.join("horus").exists() {
                    discovery.add_workspace_path(parent.to_path_buf());
                }
            }
        }
    }

    let all_plugins = discovery
        .discover_all()
        .map_err(|e| HorusError::Config(e.to_string()))?;

    println!("{}", "Available HORUS Plugins".cyan().bold());
    println!("{}", "=".repeat(50).dimmed());
    println!();

    // Group by category
    let mut by_category: std::collections::HashMap<String, Vec<crate::plugins::AvailablePlugin>> =
        std::collections::HashMap::new();

    for plugin in all_plugins {
        let cat_name = format!("{:?}", plugin.category);
        by_category.entry(cat_name).or_default().push(plugin);
    }

    for (cat, plugins) in by_category.iter() {
        println!(
            "{}",
            format!("  {} ({} plugins)", cat, plugins.len())
                .yellow()
                .bold()
        );
        for plugin in plugins {
            let source = match plugin.source {
                crate::plugins::PluginSourceType::Local => "local".yellow(),
                crate::plugins::PluginSourceType::Registry => "registry".green(),
                crate::plugins::PluginSourceType::CratesIo => "crates.io".blue(),
                crate::plugins::PluginSourceType::Git => "git".magenta(),
            };
            println!(
                "    {} {} ({})",
                plugin.name.white(),
                format!("v{}", plugin.version).dimmed(),
                source
            );
        }
        println!();
    }

    println!("{}", "Use 'horus info <name>' for details".dimmed());
    Ok(())
}

/// Show detailed info about a specific plugin
pub fn run_info(name: String) -> HorusResult<()> {
    use crate::plugins::PluginDiscovery;

    let mut discovery = PluginDiscovery::new();

    // Add local workspace
    if let Ok(cwd) = std::env::current_dir() {
        if cwd.join("horus").exists() {
            discovery.add_workspace_path(cwd);
        }
    }

    match discovery
        .get_plugin(&name)
        .map_err(|e| HorusError::Config(e.to_string()))?
    {
        Some(plugin) => {
            println!("{}", plugin.name.cyan().bold());
            println!("{}", "=".repeat(plugin.name.len()).dimmed());
            println!();
            println!("  Version:     {}", plugin.version.white());
            println!("  Category:    {:?}", plugin.category);
            println!("  Source:      {:?}", plugin.source);
            println!("  Description: {}", plugin.description);
            println!();
            println!("  {}", "Compatibility".yellow());
            println!("    HORUS:     {}", plugin.horus_compat);
            println!("    Platforms: {}", plugin.platforms.join(", "));
            println!(
                "    Prebuilt:  {}",
                if plugin.has_prebuilt { "Yes" } else { "No" }
            );
            println!();
            if !plugin.system_deps.is_empty() {
                println!("  {}", "System Dependencies".yellow());
                for dep in &plugin.system_deps {
                    println!("    - {}", dep);
                }
                println!();
            }
            if !plugin.features.is_empty() {
                println!("  {}", "Features".yellow());
                for feature in &plugin.features {
                    println!("    - {}", feature);
                }
                println!();
            }
            println!("  {}", "Installation".yellow());
            println!(
                "{}",
                discovery
                    .get_install_instructions(&plugin)
                    .lines()
                    .map(|l| format!("    {}", l))
                    .collect::<Vec<_>>()
                    .join("\n")
            );
        }
        None => {
            println!("{}", format!("Plugin '{}' not found", name).red());
            println!(
                "{}",
                "Use 'horus search <query>' to find plugins".dimmed()
            );
        }
    }
    Ok(())
}

/// Resolve the package directory after installation for plugin registration
fn resolve_package_dir(name: &str, version: &str, global: bool) -> Option<PathBuf> {
    if global {
        let home = dirs::home_dir()?;
        let versioned = home
            .join(".horus/cache")
            .join(format!("{}@{}", name, version));
        if versioned.exists() {
            return Some(versioned);
        }
        let plain = home.join(".horus/cache").join(name);
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

/// Install a plugin package from registry
///
/// Plugins default to global install since they extend the CLI tool itself.
/// Use --local to install into a specific project workspace instead.
pub fn run_install(plugin: String, ver: Option<String>, local: bool) -> HorusResult<()> {
    // Plugins default to global installation (CLI-wide)
    let global = !local;

    println!(
        "{} Installing plugin: {}{}",
        "[PLUGIN]".magenta().bold(),
        plugin.yellow(),
        ver.as_ref().map(|v| format!("@{}", v)).unwrap_or_default()
    );

    // Determine install target - plugins default to global
    let install_target = if global {
        println!("  Scope: {}", "global (~/.horus/cache/)".dimmed());
        workspace::InstallTarget::Global
    } else {
        let target = workspace::detect_or_select_workspace(true)
            .map_err(|e| HorusError::Config(e.to_string()))?;
        match &target {
            workspace::InstallTarget::Local(p) => {
                println!("  Scope: {}", format!("local ({})", p.display()).dimmed());
            }
            workspace::InstallTarget::Global => {
                println!("  Scope: {}", "global (~/.horus/cache/)".dimmed());
            }
        }
        target
    };

    // Install via the existing package install flow
    let client = registry::RegistryClient::new();
    let installed_version = client
        .install_to_target(&plugin, ver.as_deref(), install_target.clone())
        .map_err(|e| HorusError::Config(e.to_string()))?;

    // Register as CLI plugin (symlink + lock file entry)
    let is_global = matches!(install_target, workspace::InstallTarget::Global);
    let project_root = match &install_target {
        workspace::InstallTarget::Local(p) => Some(p.as_path()),
        workspace::InstallTarget::Global => None,
    };

    if let Some(pkg_dir) = resolve_package_dir(&plugin, &installed_version, is_global) {
        let source = crate::plugins::PluginSource::Registry;
        match super::pkg::register_plugin_after_install(&pkg_dir, source, is_global, project_root) {
            Ok(Some(cmd)) => {
                println!(
                    "\n  {} Registered CLI command: {}",
                    "✓".green(),
                    format!("horus {}", cmd).green()
                );
            }
            Ok(None) => {
                println!(
                    "\n  {} Package installed but no CLI plugin detected",
                    "ℹ".cyan()
                );
            }
            Err(e) => {
                println!("\n  {} Failed to register plugin: {}", "⚠".yellow(), e);
            }
        }
    }

    // Update horus.yaml with plugin-prefixed dependency
    if let workspace::InstallTarget::Local(workspace_path) = &install_target {
        let horus_yaml_path = workspace_path.join("horus.yaml");
        if horus_yaml_path.exists() {
            let dep_string = format!("plugin:{}", plugin);
            let version = ver.as_deref().unwrap_or(&installed_version);
            if let Err(e) =
                yaml_utils::add_dependency_to_horus_yaml(&horus_yaml_path, &dep_string, version)
            {
                println!("  {} Failed to update horus.yaml: {}", "⚠".yellow(), e);
            } else {
                println!("  {} Updated horus.yaml", "✓".green());
            }
        }
    }

    println!();
    println!(
        "{} Plugin {} v{} installed successfully!",
        "✓".green().bold(),
        plugin.green(),
        installed_version
    );

    Ok(())
}

/// Remove an installed plugin package
pub fn run_remove(plugin: String, global: bool) -> HorusResult<()> {
    // Plugins default to global scope
    let is_global = global || {
        // If not explicitly --global, check if it's installed globally
        let home = dirs::home_dir();
        home.map(|h| {
            let cache = h.join(".horus/cache");
            cache.join(&plugin).exists()
                || std::fs::read_dir(&cache)
                    .ok()
                    .map(|entries| {
                        entries.filter_map(|e| e.ok()).any(|e| {
                            e.file_name()
                                .to_string_lossy()
                                .starts_with(&format!("{}@", plugin))
                        })
                    })
                    .unwrap_or(false)
        })
        .unwrap_or(false)
    };

    println!(
        "{} Removing plugin: {}",
        "[PLUGIN]".magenta().bold(),
        plugin.yellow()
    );

    // Unregister the CLI plugin (symlink + lock file) before removing files
    let project_root = if !is_global {
        workspace::find_workspace_root()
    } else {
        None
    };

    // Try to unregister - the plugin command name might differ from package name
    // Convention: package "horus-foo" provides command "foo"
    let command_name = plugin.strip_prefix("horus-").unwrap_or(&plugin).to_string();

    if let Err(e) = super::pkg::unregister_plugin(&command_name, is_global, project_root.as_deref())
    {
        // Not fatal - plugin might not have had a CLI extension
        println!(
            "  {} Plugin CLI cleanup: {}",
            "ℹ".dimmed(),
            e.to_string().dimmed()
        );
    }

    // Delegate file removal to pkg::run_remove
    super::pkg::run_remove(plugin, is_global, None)
}
