//! Plugin command - search, discover, and inspect HORUS plugins

use colored::*;
use horus_core::error::{HorusError, HorusResult};

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

    println!("{}", "Use 'horus plugins info <name>' for details".dimmed());
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
                "Use 'horus plugins search <query>' to find plugins".dimmed()
            );
        }
    }
    Ok(())
}
