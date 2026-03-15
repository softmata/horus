//! Plugin command - search, discover, inspect, install, and remove HORUS plugins

use crate::manifest::HORUS_TOML;
use crate::{cli_output, registry, workspace};
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::path::PathBuf;

/// Search for plugins by query with optional category filter
pub fn run_search_with_category(
    query: String,
    category: Option<String>,
    json: bool,
) -> HorusResult<()> {
    use crate::plugins::PluginDiscovery;

    let mut discovery = PluginDiscovery::new();
    add_local_workspace(&mut discovery);

    let all = discovery
        .discover_all()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    let query_lower = query.to_lowercase();
    let category_filter = category.as_ref().and_then(|c| parse_category(c));

    let results: Vec<_> = all
        .into_iter()
        .filter(|p| {
            let matches_query = p.name.to_lowercase().contains(&query_lower)
                || p.description.to_lowercase().contains(&query_lower)
                || p.features
                    .iter()
                    .any(|f| f.to_lowercase().contains(&query_lower));
            let matches_category = category_filter
                .as_ref()
                .map(|cat| p.category == *cat)
                .unwrap_or(true);
            matches_query && matches_category
        })
        .collect();

    if json {
        let items: Vec<_> = results
            .iter()
            .map(|p| {
                serde_json::json!({
                    "name": p.name,
                    "description": p.description,
                    "category": format!("{}", p.category),
                    "features": p.features,
                })
            })
            .collect();
        let output = serde_json::json!({
            "query": query,
            "category": category,
            "results": items,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    print_search_results(&results, &query, category.as_deref());
    Ok(())
}

/// Search for plugins by query (no category filter)
pub fn run_search(query: String) -> HorusResult<()> {
    run_search_with_category(query, None, false)
}

/// Parse a category string into PluginCategory
fn parse_category(s: &str) -> Option<crate::plugins::PluginCategory> {
    use crate::plugins::PluginCategory;
    match s.to_lowercase().as_str() {
        "camera" => Some(PluginCategory::Camera),
        "lidar" => Some(PluginCategory::Lidar),
        "imu" => Some(PluginCategory::Imu),
        "motor" => Some(PluginCategory::Motor),
        "servo" => Some(PluginCategory::Servo),
        "bus" => Some(PluginCategory::Bus),
        "gps" => Some(PluginCategory::Gps),
        "force_torque" | "forcetorque" | "force-torque" => Some(PluginCategory::ForceTorque),
        "simulation" | "sim" => Some(PluginCategory::Simulation),
        "cli" => Some(PluginCategory::Cli),
        "other" => Some(PluginCategory::Other),
        _ => None,
    }
}

/// Add local workspace paths to discovery
fn add_local_workspace(discovery: &mut crate::plugins::PluginDiscovery) {
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

/// Print search results
fn print_search_results(
    results: &[crate::plugins::AvailablePlugin],
    query: &str,
    category: Option<&str>,
) {
    let label = if let Some(cat) = category {
        format!("'{}' in category '{}'", query, cat)
    } else {
        format!("'{}'", query)
    };

    if results.is_empty() {
        println!(
            "{}",
            format!("No plugins found matching {}", label).yellow()
        );
        if category.is_some() {
            println!(
                "{}",
                "Try without --category or use a different category".dimmed()
            );
            println!(
                "{}",
                "Categories: camera, lidar, imu, motor, servo, bus, gps, force-torque, simulation, cli, other".dimmed()
            );
        }
    } else {
        println!(
            "{}",
            format!("Found {} plugins matching {}:", results.len(), label)
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
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
    {
        Some(plugin) => {
            println!("{}", plugin.name.cyan().bold());
            println!("{}", "=".repeat(plugin.name.len()).dimmed());
            println!();
            println!("  Version:     {}", plugin.version.white());
            println!("  Category:    {}", plugin.category);
            println!("  Source:      {}", plugin.source);
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
            println!("{}", "Use 'horus search <query>' to find plugins".dimmed());
        }
    }
    Ok(())
}

/// Unified info command — checks both plugin discovery and installed packages
pub fn run_info_unified(name: String, json: bool) -> HorusResult<()> {
    use crate::plugins::PluginDiscovery;

    let mut discovery = PluginDiscovery::new();
    add_local_workspace(&mut discovery);

    // Try plugin discovery first
    if let Ok(Some(plugin)) = discovery.get_plugin(&name) {
        if json {
            let output = serde_json::json!({
                "name": plugin.name,
                "description": plugin.description,
                "source": "plugin",
                "category": format!("{}", plugin.category),
                "features": plugin.features,
            });
            println!(
                "{}",
                serde_json::to_string_pretty(&output).unwrap_or_default()
            );
            return Ok(());
        }
        return run_info(name);
    }

    // Fall back to registry search for installed packages
    let client = registry::RegistryClient::new();
    match client.search(&name, None, None) {
        Ok(results) => {
            // Look for exact match
            if let Some(pkg) = results.iter().find(|p| p.name == name) {
                if json {
                    let output = serde_json::json!({
                        "name": pkg.name,
                        "version": pkg.version,
                        "description": pkg.description,
                        "source": "registry",
                    });
                    println!(
                        "{}",
                        serde_json::to_string_pretty(&output).unwrap_or_default()
                    );
                    return Ok(());
                }
                println!("{}", pkg.name.cyan().bold());
                println!("{}", "=".repeat(pkg.name.len()).dimmed());
                println!();
                println!("  Version:     {}", pkg.version.white());
                println!(
                    "  Description: {}",
                    pkg.description.as_deref().unwrap_or("No description")
                );
                println!("  Source:      {}", "registry".green());
                println!();
                println!("  {}", "Installation".yellow());
                println!("    horus install {}", pkg.name);
                return Ok(());
            }
        }
        Err(_) => {
            // Registry unreachable, skip
        }
    }

    // Check local installed packages
    let check_dirs: Vec<std::path::PathBuf> = {
        let mut dirs = Vec::new();
        if let Some(root) = workspace::find_workspace_root() {
            dirs.push(root.join(".horus/packages").join(&name));
        }
        if let Ok(cache) = crate::paths::cache_dir() {
            dirs.push(cache.join(&name));
            // Also check versioned directories
            if let Ok(entries) = std::fs::read_dir(&cache) {
                for entry in entries.flatten() {
                    let entry_name = entry.file_name().to_string_lossy().to_string();
                    if entry_name.starts_with(&format!("{}@", name)) {
                        dirs.push(entry.path());
                    }
                }
            }
        }
        dirs
    };

    for dir in &check_dirs {
        if dir.exists() {
            let display_name = dir
                .file_name()
                .unwrap_or_default()
                .to_string_lossy()
                .to_string();

            if json {
                let mut info = serde_json::json!({
                    "name": display_name,
                    "location": dir.display().to_string(),
                    "source": "local",
                    "status": "installed",
                });
                let toml_path = dir.join(HORUS_TOML);
                if toml_path.exists() {
                    if let Ok(content) = std::fs::read_to_string(&toml_path) {
                        if let Ok(parsed) = toml::from_str::<toml::Value>(&content) {
                            if let Some(desc) = parsed.get("description").and_then(|v| v.as_str()) {
                                info["description"] = serde_json::Value::String(desc.to_string());
                            }
                            if let Some(ver) = parsed.get("version").and_then(|v| v.as_str()) {
                                info["version"] = serde_json::Value::String(ver.to_string());
                            }
                        }
                    }
                }
                println!(
                    "{}",
                    serde_json::to_string_pretty(&info).unwrap_or_default()
                );
                return Ok(());
            }

            println!("{}", display_name.cyan().bold());
            println!("{}", "=".repeat(display_name.len()).dimmed());
            println!();
            println!("  Location:  {}", dir.display());
            println!("  Status:    {}", "installed".green());

            // Try to read horus.toml or package.json for more info
            let toml_path = dir.join(HORUS_TOML);
            if toml_path.exists() {
                if let Ok(content) = std::fs::read_to_string(&toml_path) {
                    if let Ok(parsed) = toml::from_str::<toml::Value>(&content) {
                        if let Some(desc) = parsed.get("description").and_then(|v| v.as_str()) {
                            println!("  Description: {}", desc);
                        }
                        if let Some(ver) = parsed.get("version").and_then(|v| v.as_str()) {
                            println!("  Version:     {}", ver.white());
                        }
                    }
                }
            }
            return Ok(());
        }
    }

    // Nothing found
    if json {
        let output = serde_json::json!({
            "name": name,
            "found": false,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Err(HorusError::Config(ConfigError::Other(format!(
            "'{}' not found",
            name
        ))));
    }
    Err(HorusError::Config(ConfigError::Other(format!(
        "'{}' not found. Use 'horus search <query>' to find available packages and plugins",
        name
    ))))
}

/// Resolve the package directory after installation for plugin registration
/// Resolve package directory — delegates to shared implementation in pkg module.
fn resolve_package_dir(name: &str, version: &str, global: bool) -> Option<PathBuf> {
    super::pkg::resolve_installed_package_dir(name, version, global)
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
        cli_output::ICON_INFO.magenta().bold(),
        plugin.yellow(),
        ver.as_ref().map(|v| format!("@{}", v)).unwrap_or_default()
    );

    // Determine install target - plugins default to global
    let install_target = if global {
        println!("  Scope: {}", "global (user cache)".dimmed());
        workspace::InstallTarget::Global
    } else {
        let target = workspace::detect_or_select_workspace(true)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
        match &target {
            workspace::InstallTarget::Local(p) => {
                println!("  Scope: {}", format!("local ({})", p.display()).dimmed());
            }
            workspace::InstallTarget::Global => {
                println!("  Scope: {}", "global (user cache)".dimmed());
            }
        }
        target
    };

    // Install via the existing package install flow
    let client = registry::RegistryClient::new();
    let installed_version = client
        .install_to_target(&plugin, ver.as_deref(), install_target.clone())
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

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
                    cli_output::ICON_SUCCESS.green(),
                    format!("horus {}", cmd).green()
                );
            }
            Ok(None) => {
                println!(
                    "\n  {} Package installed but no CLI plugin detected",
                    cli_output::ICON_INFO.cyan()
                );
            }
            Err(e) => {
                println!(
                    "\n  {} Failed to register plugin: {}",
                    cli_output::ICON_WARN.yellow(),
                    e
                );
            }
        }
    }

    // Plugin registration is handled by the plugin registry (lock file + symlink) above.
    // Dependencies are tracked in horus.toml [dependencies] and managed via `horus add`.

    println!();
    println!(
        "{} Plugin {} v{} installed successfully!",
        cli_output::ICON_SUCCESS.green().bold(),
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
        cli_output::ICON_INFO.magenta().bold(),
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
            cli_output::ICON_HINT.dimmed(),
            e.to_string().dimmed()
        );
    }

    // Delegate file removal to pkg::run_remove
    super::pkg::run_remove(plugin, is_global, None)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::plugins::{PluginCategory, PluginSourceType};
    use std::fs;
    use tempfile::TempDir;

    // ── parse_category ──────────────────────────────────────────────────

    #[test]
    fn test_parse_category_camera() {
        assert_eq!(parse_category("camera"), Some(PluginCategory::Camera));
        assert_eq!(parse_category("Camera"), Some(PluginCategory::Camera));
        assert_eq!(parse_category("CAMERA"), Some(PluginCategory::Camera));
    }

    #[test]
    fn test_parse_category_lidar() {
        assert_eq!(parse_category("lidar"), Some(PluginCategory::Lidar));
        assert_eq!(parse_category("Lidar"), Some(PluginCategory::Lidar));
    }

    #[test]
    fn test_parse_category_imu() {
        assert_eq!(parse_category("imu"), Some(PluginCategory::Imu));
        assert_eq!(parse_category("IMU"), Some(PluginCategory::Imu));
    }

    #[test]
    fn test_parse_category_motor() {
        assert_eq!(parse_category("motor"), Some(PluginCategory::Motor));
    }

    #[test]
    fn test_parse_category_servo() {
        assert_eq!(parse_category("servo"), Some(PluginCategory::Servo));
    }

    #[test]
    fn test_parse_category_bus() {
        assert_eq!(parse_category("bus"), Some(PluginCategory::Bus));
    }

    #[test]
    fn test_parse_category_gps() {
        assert_eq!(parse_category("gps"), Some(PluginCategory::Gps));
    }

    #[test]
    fn test_parse_category_force_torque_variants() {
        assert_eq!(
            parse_category("force_torque"),
            Some(PluginCategory::ForceTorque)
        );
        assert_eq!(
            parse_category("forcetorque"),
            Some(PluginCategory::ForceTorque)
        );
        assert_eq!(
            parse_category("force-torque"),
            Some(PluginCategory::ForceTorque)
        );
        assert_eq!(
            parse_category("Force_Torque"),
            Some(PluginCategory::ForceTorque)
        );
    }

    #[test]
    fn test_parse_category_simulation() {
        assert_eq!(
            parse_category("simulation"),
            Some(PluginCategory::Simulation)
        );
        assert_eq!(parse_category("sim"), Some(PluginCategory::Simulation));
        assert_eq!(parse_category("SIM"), Some(PluginCategory::Simulation));
    }

    #[test]
    fn test_parse_category_cli() {
        assert_eq!(parse_category("cli"), Some(PluginCategory::Cli));
    }

    #[test]
    fn test_parse_category_other() {
        assert_eq!(parse_category("other"), Some(PluginCategory::Other));
    }

    #[test]
    fn test_parse_category_unknown_returns_none() {
        assert_eq!(parse_category("nonexistent"), None);
        assert_eq!(parse_category(""), None);
        assert_eq!(parse_category("robot"), None);
        assert_eq!(parse_category("sensor"), None);
    }

    // ── print_search_results ────────────────────────────────────────────

    #[test]
    fn test_print_search_results_empty_no_category() {
        // Should not panic with empty results and no category
        print_search_results(&[], "test-query", None);
    }

    #[test]
    fn test_print_search_results_empty_with_category() {
        // Should not panic; prints category hint
        print_search_results(&[], "test-query", Some("camera"));
    }

    #[test]
    fn test_print_search_results_with_results_no_category() {
        use crate::plugins::AvailablePlugin;

        let plugins = vec![AvailablePlugin {
            name: "horus-test-cam".to_string(),
            version: "0.1.0".to_string(),
            description: "A test camera plugin".to_string(),
            category: PluginCategory::Camera,
            source: PluginSourceType::Local,
            platforms: vec!["linux-x86_64".to_string()],
            horus_compat: ">=0.1.0".to_string(),
            has_prebuilt: false,
            system_deps: vec![],
            features: vec!["rgb".to_string(), "depth".to_string()],
        }];

        // Should not panic
        print_search_results(&plugins, "cam", None);
    }

    #[test]
    fn test_print_search_results_with_prebuilt() {
        use crate::plugins::AvailablePlugin;

        let plugins = vec![AvailablePlugin {
            name: "horus-prebuilt".to_string(),
            version: "1.0.0".to_string(),
            description: "Prebuilt plugin".to_string(),
            category: PluginCategory::Motor,
            source: PluginSourceType::Registry,
            platforms: vec!["linux-x86_64".to_string()],
            horus_compat: ">=0.1.0".to_string(),
            has_prebuilt: true,
            system_deps: vec![],
            features: vec![],
        }];

        print_search_results(&plugins, "prebuilt", None);
    }

    #[test]
    fn test_print_search_results_all_source_types() {
        use crate::plugins::AvailablePlugin;

        let sources = [
            PluginSourceType::Local,
            PluginSourceType::Registry,
            PluginSourceType::CratesIo,
            PluginSourceType::Git,
        ];

        for source in &sources {
            let plugins = vec![AvailablePlugin {
                name: format!("horus-{:?}", source),
                version: "0.1.0".to_string(),
                description: "Test".to_string(),
                category: PluginCategory::Other,
                source: source.clone(),
                platforms: vec![],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: false,
                system_deps: vec![],
                features: vec![],
            }];
            print_search_results(&plugins, "test", Some("other"));
        }
    }

    #[test]
    fn test_print_search_results_no_features() {
        use crate::plugins::AvailablePlugin;

        let plugins = vec![AvailablePlugin {
            name: "horus-no-features".to_string(),
            version: "0.1.0".to_string(),
            description: "No features".to_string(),
            category: PluginCategory::Other,
            source: PluginSourceType::Local,
            platforms: vec![],
            horus_compat: ">=0.1.0".to_string(),
            has_prebuilt: false,
            system_deps: vec![],
            features: vec![],
        }];

        // Should not print "Features:" line
        print_search_results(&plugins, "no-features", None);
    }

    #[test]
    fn test_print_search_results_multiple_plugins() {
        use crate::plugins::AvailablePlugin;

        let plugins = vec![
            AvailablePlugin {
                name: "horus-a".to_string(),
                version: "0.1.0".to_string(),
                description: "Plugin A".to_string(),
                category: PluginCategory::Camera,
                source: PluginSourceType::Local,
                platforms: vec![],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: false,
                system_deps: vec![],
                features: vec!["feat-a".to_string()],
            },
            AvailablePlugin {
                name: "horus-b".to_string(),
                version: "0.2.0".to_string(),
                description: "Plugin B".to_string(),
                category: PluginCategory::Lidar,
                source: PluginSourceType::Registry,
                platforms: vec!["linux-x86_64".to_string()],
                horus_compat: ">=0.1.0".to_string(),
                has_prebuilt: true,
                system_deps: vec![],
                features: vec![],
            },
        ];

        print_search_results(&plugins, "horus", None);
    }

    // ── run_search ──────────────────────────────────────────────────────

    #[test]
    fn test_run_search_delegates_to_search_with_category() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        // Create a temp dir that does NOT look like a workspace
        // (no `horus` subdir) so add_local_workspace is a no-op
        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // run_search should succeed (discovers known plugins and filters)
        let result = run_search("camera".to_string());
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── run_search_with_category ────────────────────────────────────────

    #[test]
    fn test_search_with_category_no_match() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // Search for something that matches no known plugin
        let result = run_search_with_category(
            "zzz_nonexistent_zzz".to_string(),
            None,
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_with_category_filter() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // Known plugins include "horus-rplidar" in Lidar category
        let result = run_search_with_category(
            "rplidar".to_string(),
            Some("lidar".to_string()),
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_with_category_mismatch() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // Search for rplidar but in camera category — should find nothing
        let result = run_search_with_category(
            "rplidar".to_string(),
            Some("camera".to_string()),
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_with_invalid_category_matches_all() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // Invalid category parses to None — acts as if no category filter
        let result = run_search_with_category(
            "rplidar".to_string(),
            Some("invalid_cat".to_string()),
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_json_output() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // JSON output path
        let result = run_search_with_category(
            "realsense".to_string(),
            None,
            true,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_json_with_category() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_search_with_category(
            "realsense".to_string(),
            Some("camera".to_string()),
            true,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_by_feature_name() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // "pointcloud" is a feature of horus-realsense
        let result = run_search_with_category(
            "pointcloud".to_string(),
            None,
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_search_case_insensitive() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // Uppercase query should still match lowercase plugin names
        let result = run_search_with_category(
            "RPLIDAR".to_string(),
            None,
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── run_search_with_category + local workspace ──────────────────────

    #[test]
    fn test_search_discovers_local_workspace_plugins() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // Create a workspace layout: temp/horus/ exists => add_local_workspace adds it
        let horus_dir = temp.path().join("horus");
        fs::create_dir_all(&horus_dir).unwrap();

        // Create a local plugin: temp/horus-test-local/Cargo.toml
        let plugin_dir = temp.path().join("horus-test-local");
        fs::create_dir_all(&plugin_dir).unwrap();
        let cargo_content = r#"
[package]
name = "horus-test-local"
version = "0.1.0"
description = "A local test plugin for unit testing"

[features]
default = []
test-feature = []
"#;
        fs::write(plugin_dir.join("Cargo.toml"), cargo_content).unwrap();

        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_search_with_category(
            "test-local".to_string(),
            None,
            false,
        );
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── add_local_workspace ─────────────────────────────────────────────

    #[test]
    fn test_add_local_workspace_with_horus_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // Create the `horus` dir in temp so the condition matches
        fs::create_dir_all(temp.path().join("horus")).unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let mut discovery = crate::plugins::PluginDiscovery::new();
        add_local_workspace(&mut discovery);

        // Discovery should have one workspace path added
        // We verify indirectly: discover_local on temp.path() succeeds
        let result = discovery.discover_local(temp.path());
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_add_local_workspace_parent_has_horus_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // Create temp/horus/ and temp/subdir/
        fs::create_dir_all(temp.path().join("horus")).unwrap();
        let subdir = temp.path().join("subdir");
        fs::create_dir_all(&subdir).unwrap();

        // CWD = temp/subdir/  (parent has `horus`)
        std::env::set_current_dir(&subdir).unwrap();

        let mut discovery = crate::plugins::PluginDiscovery::new();
        add_local_workspace(&mut discovery);

        // Should not panic
        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_add_local_workspace_no_horus_dir() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // No `horus` subdir — workspace should not be added
        std::env::set_current_dir(temp.path()).unwrap();

        let mut discovery = crate::plugins::PluginDiscovery::new();
        add_local_workspace(&mut discovery);
        // Should not panic; just no workspace added

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── run_info ────────────────────────────────────────────────────────

    #[test]
    fn test_run_info_known_plugin() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // horus-realsense is a known plugin — should succeed and print info
        let result = run_info("horus-realsense".to_string());
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unknown_plugin() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // Unknown plugin — should still succeed (prints "not found" message)
        let result = run_info("nonexistent-plugin-zzz".to_string());
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_plugin_with_system_deps() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        // horus-realsense has system_deps ["librealsense2-dev"]
        let result = run_info("horus-realsense".to_string());
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── run_info_unified ────────────────────────────────────────────────

    #[test]
    fn test_run_info_unified_known_plugin_text() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_info_unified("horus-rplidar".to_string(), false);
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unified_known_plugin_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_info_unified("horus-rplidar".to_string(), true);
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unified_not_found_text() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_info_unified("zzz-totally-nonexistent-zzz".to_string(), false);
        assert!(result.is_err());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unified_not_found_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_info_unified("zzz-totally-nonexistent-zzz".to_string(), true);
        assert!(result.is_err());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unified_local_package_with_horus_toml() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // Create a workspace with .horus/packages/my-local-pkg/horus.toml
        let pkg_dir = temp.path().join(".horus/packages/my-local-pkg");
        fs::create_dir_all(&pkg_dir).unwrap();

        let toml_content = r#"
[package]
name = "my-local-pkg"

description = "A locally installed package"
version = "1.2.3"
"#;
        fs::write(pkg_dir.join("horus.toml"), toml_content).unwrap();

        // Also create horus.toml at workspace root so find_workspace_root finds it
        fs::write(temp.path().join("horus.toml"), "[package]\nname = \"ws\"\n").unwrap();

        std::env::set_current_dir(temp.path()).unwrap();

        // "my-local-pkg" is not a known plugin; registry is unreachable;
        // but it exists as a local installed package
        let result = run_info_unified("my-local-pkg".to_string(), false);
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unified_local_package_json() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        let pkg_dir = temp.path().join(".horus/packages/local-json-pkg");
        fs::create_dir_all(&pkg_dir).unwrap();

        let toml_content = r#"
description = "JSON test package"
version = "2.0.0"
"#;
        fs::write(pkg_dir.join("horus.toml"), toml_content).unwrap();
        fs::write(temp.path().join("horus.toml"), "[package]\nname = \"ws\"\n").unwrap();

        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_info_unified("local-json-pkg".to_string(), true);
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_run_info_unified_local_package_no_horus_toml() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // Package dir exists but has no horus.toml
        let pkg_dir = temp.path().join(".horus/packages/bare-pkg");
        fs::create_dir_all(&pkg_dir).unwrap();
        fs::write(temp.path().join("horus.toml"), "[package]\nname = \"ws\"\n").unwrap();

        std::env::set_current_dir(temp.path()).unwrap();

        let result = run_info_unified("bare-pkg".to_string(), false);
        assert!(result.is_ok());

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── resolve_package_dir ─────────────────────────────────────────────

    #[test]
    fn test_resolve_package_dir_global_versioned() {
        let home = dirs::home_dir().expect("no home dir");
        let cache = home.join(".horus/cache");
        let versioned = cache.join("test-resolve-pkg@9.9.9");

        // Create temp versioned dir
        fs::create_dir_all(&versioned).unwrap();

        let result = resolve_package_dir("test-resolve-pkg", "9.9.9", true);
        assert_eq!(result, Some(versioned.clone()));

        // Cleanup
        let _ = fs::remove_dir_all(&versioned);
    }

    #[test]
    fn test_resolve_package_dir_global_plain() {
        let home = dirs::home_dir().expect("no home dir");
        let cache = home.join(".horus/cache");
        let plain = cache.join("test-resolve-plain");

        fs::create_dir_all(&plain).unwrap();

        // No versioned dir — falls back to plain name
        let result = resolve_package_dir("test-resolve-plain", "0.0.0", true);
        assert_eq!(result, Some(plain.clone()));

        // Cleanup
        let _ = fs::remove_dir_all(&plain);
    }

    #[test]
    fn test_resolve_package_dir_global_not_found() {
        let result = resolve_package_dir("zzz-never-exists-zzz", "0.0.0", true);
        assert_eq!(result, None);
    }

    #[test]
    fn test_resolve_package_dir_local() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();

        // Create workspace with .horus/packages/my-pkg
        let pkg_dir = temp.path().join(".horus/packages/my-pkg");
        fs::create_dir_all(&pkg_dir).unwrap();

        std::env::set_current_dir(temp.path()).unwrap();

        let result = resolve_package_dir("my-pkg", "0.1.0", false);
        assert_eq!(result, Some(pkg_dir));

        std::env::set_current_dir(&prev).unwrap();
    }

    #[test]
    fn test_resolve_package_dir_local_not_found() {
        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());

        let temp = TempDir::new().unwrap();
        let prev = std::env::current_dir().unwrap();
        std::env::set_current_dir(temp.path()).unwrap();

        let result = resolve_package_dir("nonexistent-pkg", "0.1.0", false);
        assert_eq!(result, None);

        std::env::set_current_dir(&prev).unwrap();
    }

    // ── run_remove command name derivation ──────────────────────────────

    #[test]
    fn test_strip_prefix_horus_dash() {
        // The run_remove function strips "horus-" prefix to derive command name
        let name = "horus-foo-bar";
        let command_name = name.strip_prefix("horus-").unwrap_or(name);
        assert_eq!(command_name, "foo-bar");
    }

    #[test]
    fn test_strip_prefix_no_horus_prefix() {
        let name = "my-plugin";
        let command_name = name.strip_prefix("horus-").unwrap_or(name);
        assert_eq!(command_name, "my-plugin");
    }

    #[test]
    fn test_strip_prefix_exact_horus() {
        let name = "horus-";
        let command_name = name.strip_prefix("horus-").unwrap_or(name);
        assert_eq!(command_name, "");
    }

    // ── parse_category exhaustive ───────────────────────────────────────

    #[test]
    fn test_parse_category_all_valid_categories() {
        let cases = vec![
            ("camera", PluginCategory::Camera),
            ("lidar", PluginCategory::Lidar),
            ("imu", PluginCategory::Imu),
            ("motor", PluginCategory::Motor),
            ("servo", PluginCategory::Servo),
            ("bus", PluginCategory::Bus),
            ("gps", PluginCategory::Gps),
            ("force_torque", PluginCategory::ForceTorque),
            ("forcetorque", PluginCategory::ForceTorque),
            ("force-torque", PluginCategory::ForceTorque),
            ("simulation", PluginCategory::Simulation),
            ("sim", PluginCategory::Simulation),
            ("cli", PluginCategory::Cli),
            ("other", PluginCategory::Other),
        ];

        for (input, expected) in cases {
            assert_eq!(
                parse_category(input),
                Some(expected),
                "parse_category({:?}) mismatch",
                input
            );
        }
    }

    #[test]
    fn test_parse_category_mixed_case() {
        // All inputs are lowercased internally
        let cases = vec![
            "Camera", "CAMERA", "cAmErA", "Lidar", "LIDAR", "IMU", "Imu",
            "MOTOR", "Motor", "SERVO", "BUS", "GPS", "FORCE_TORQUE",
            "ForCeTorQue", "FORCE-TORQUE", "SIMULATION", "SIM", "CLI", "OTHER",
        ];

        for input in cases {
            assert!(
                parse_category(input).is_some(),
                "parse_category({:?}) should return Some",
                input
            );
        }
    }
}
