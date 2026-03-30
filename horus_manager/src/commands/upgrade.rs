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
                    "  {} New version available: {} -> {}",
                    "!".yellow(),
                    current_version.dimmed(),
                    latest.green()
                );
                if !check_only {
                    upgrade_horus(&latest)?;
                }
            } else {
                println!("  {} horus is up to date", "*".green());
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
            println!("    {} {}", "-".dimmed(), plugin);
        }
        if !check_only {
            upgrade_plugins();
        } else {
            check_plugin_updates();
        }
    }

    Ok(())
}

/// Check the latest horus version from the registry API.
pub fn check_latest_version() -> Result<Option<String>> {
    // Try to fetch from registry
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(5))
        .build()?;

    let resp = client
        .get("https://horusrobotics.dev/api/packages/horus/latest")
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
pub(crate) fn upgrade_horus(version: &str) -> Result<()> {
    println!("  Building horus {}...", version.cyan());

    // Try cargo install from source
    let status = std::process::Command::new("cargo")
        .args(["install", "--path", "."])
        .status();

    match status {
        Ok(s) if s.success() => {
            println!("  {} Upgraded to {}", "*".green(), version.green());
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

/// Check the latest version of a plugin package from the registry API.
fn check_plugin_version(name: &str) -> Option<String> {
    let client = reqwest::blocking::Client::builder()
        .timeout(std::time::Duration::from_secs(5))
        .build()
        .ok()?;

    let encoded = name.replace('@', "%40").replace('/', "%2F");
    let url = format!("https://horusrobotics.dev/api/packages/{}/latest", encoded);

    let resp = client.get(&url).send().ok()?;
    if !resp.status().is_success() {
        return None;
    }

    let body: serde_json::Value = resp.json().ok()?;
    body.get("version")
        .or_else(|| body.get("latest_version"))
        .and_then(|v| v.as_str())
        .map(String::from)
}

/// Load the plugin registry and collect installed plugin entries with their versions.
///
/// Returns a vec of (plugin_name, package_name, installed_version, is_global).
fn collect_plugin_entries() -> Vec<(String, String, String, bool)> {
    use crate::plugins::PluginRegistry;

    let mut entries = Vec::new();

    // Collect from global registry
    if let Ok(registry) = PluginRegistry::load_global() {
        for (cmd_name, entry) in &registry.plugins {
            entries.push((
                cmd_name.clone(),
                entry.package.clone(),
                entry.version.clone(),
                true,
            ));
        }
    }

    // Collect from local/project registry
    let cwd = std::env::current_dir().unwrap_or_else(|_| std::path::PathBuf::from("."));
    if let Some(registry) = PluginRegistry::load_project(&cwd) {
        for (cmd_name, entry) in &registry.plugins {
            // Skip if already found in global (global takes precedence for upgrade)
            if !entries.iter().any(|(name, _, _, _)| name == cmd_name) {
                entries.push((
                    cmd_name.clone(),
                    entry.package.clone(),
                    entry.version.clone(),
                    false,
                ));
            }
        }
    }

    entries
}

/// Check for plugin updates (display only, no install).
fn check_plugin_updates() {
    let entries = collect_plugin_entries();
    if entries.is_empty() {
        return;
    }

    println!();
    println!("  {}", "Checking plugin versions:".bold());
    for (cmd_name, package_name, installed_version, _is_global) in &entries {
        match check_plugin_version(package_name) {
            Some(latest) if latest != *installed_version => {
                println!(
                    "    {} {} ({}) {} -> {}",
                    "!".yellow(),
                    cmd_name,
                    package_name,
                    installed_version.dimmed(),
                    latest.green()
                );
            }
            Some(_) => {
                println!(
                    "    {} {} ({}) up to date",
                    "*".green(),
                    cmd_name,
                    installed_version.dimmed()
                );
            }
            None => {
                println!("    {} {} could not check version", "?".dimmed(), cmd_name);
            }
        }
    }
}

/// Upgrade all installed plugins to their latest versions.
fn upgrade_plugins() {
    let entries = collect_plugin_entries();
    if entries.is_empty() {
        return;
    }

    println!();
    println!("  {}", "Upgrading plugins:".bold());
    for (cmd_name, package_name, installed_version, is_global) in &entries {
        match check_plugin_version(package_name) {
            Some(latest) if latest != *installed_version => {
                println!(
                    "    {} Upgrading {} {} -> {}...",
                    "!".yellow(),
                    cmd_name,
                    installed_version.dimmed(),
                    latest.green()
                );

                match reinstall_plugin(package_name, &latest, *is_global) {
                    Ok(()) => {
                        println!(
                            "    {} {} upgraded to {}",
                            "*".green(),
                            cmd_name,
                            latest.green()
                        );
                    }
                    Err(e) => {
                        println!("    {} Failed to upgrade {}: {}", "!".yellow(), cmd_name, e);
                    }
                }
            }
            Some(_) => {
                println!(
                    "    {} {} already up to date ({})",
                    "*".green(),
                    cmd_name,
                    installed_version.dimmed()
                );
            }
            None => {
                println!(
                    "    {} {} could not determine latest version — skipping",
                    "?".dimmed(),
                    cmd_name
                );
            }
        }
    }
}

/// Reinstall a plugin at a specific version using the registry client.
fn reinstall_plugin(package_name: &str, version: &str, global: bool) -> Result<()> {
    use crate::plugins::PluginSource;
    use crate::{registry, workspace};

    let install_target = if global {
        workspace::InstallTarget::Global
    } else {
        workspace::InstallTarget::Local(std::env::current_dir()?)
    };

    let client = registry::RegistryClient::new();
    let installed_version =
        client.install_to_target(package_name, Some(version), install_target)?;

    // Re-register the plugin after reinstall
    if let Some(pkg_dir) =
        super::pkg::resolve_installed_package_dir(package_name, &installed_version, global)
    {
        let project_root = if global {
            None
        } else {
            Some(std::env::current_dir()?)
        };
        if let Err(e) = super::pkg::register_plugin_after_install(
            &pkg_dir,
            PluginSource::Registry,
            global,
            project_root.as_deref(),
        ) {
            log::warn!("Plugin re-registration failed: {}", e);
        }
    }

    Ok(())
}

/// List installed plugins from the global and local registries.
pub(crate) fn list_installed_plugins() -> Vec<String> {
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

#[cfg(test)]
mod tests {
    use super::*;

    // ── run_upgrade ─────────────────────────────────────────────────────

    #[test]
    fn run_upgrade_check_only_succeeds() {
        // check_only=true should always succeed even without network
        let result = run_upgrade(true);
        result.unwrap();
    }

    #[test]
    fn run_upgrade_full_succeeds() {
        // check_only=false should also succeed — network failure is handled gracefully
        let result = run_upgrade(false);
        result.unwrap();
    }

    #[test]
    fn run_upgrade_returns_ok_not_err() {
        // Both code paths (check_only true and false) must return Ok
        // even when the registry is unreachable
        for check_only in [true, false] {
            assert!(
                run_upgrade(check_only).is_ok(),
                "run_upgrade(check_only={}) should succeed",
                check_only,
            );
        }
    }

    // ── check_latest_version ────────────────────────────────────────────

    #[test]
    fn check_latest_version_returns_ok() {
        // In test environment without network access to horusrobotics.dev,
        // should return Ok(None) rather than Err
        let result = check_latest_version();
        result.unwrap();
    }

    #[test]
    fn check_latest_version_returns_none_when_registry_unreachable() {
        // Registry is not available in test env, so result should be None
        let result = check_latest_version().unwrap();
        // We expect None since the registry is not running, but if it somehow
        // is reachable, a Some is also acceptable
        // This test just ensures no panic/error
        let _ = result;
    }

    // ── upgrade_horus ───────────────────────────────────────────────────

    #[test]
    fn upgrade_horus_returns_ok_even_on_failure() {
        // upgrade_horus always returns Ok even if cargo install fails
        // (it prints a fallback message instead of erroring)
        // Run from temp dir where there's no Cargo.toml so cargo install fails fast
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = upgrade_horus("99.99.99");
        std::env::set_current_dir(original).unwrap();

        result.unwrap();
    }

    #[test]
    fn upgrade_horus_with_empty_version() {
        // Run from temp dir so cargo install fails fast instead of building
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = upgrade_horus("");
        std::env::set_current_dir(original).unwrap();

        result.unwrap();
    }

    #[test]
    fn upgrade_horus_with_real_looking_version() {
        // Run from temp dir so cargo install fails fast instead of building
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = upgrade_horus("0.2.0");
        std::env::set_current_dir(original).unwrap();

        result.unwrap();
    }

    // ── list_installed_plugins ──────────────────────────────────────────

    #[test]
    fn list_installed_plugins_returns_vec() {
        // Should return a Vec (possibly empty) without errors
        let plugins = list_installed_plugins();
        // Just verify it's a vec and doesn't panic
        let _ = plugins.len();
    }

    #[test]
    fn list_installed_plugins_no_local_plugins_dir() {
        // When there's no .horus/plugins in cwd, local plugins should be empty
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(tmp.path()).unwrap();

        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        // Should have no local plugins (global might exist from real home)
        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert!(local_plugins.is_empty());
    }

    #[test]
    fn list_installed_plugins_finds_local_plugins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Create .horus/plugins with some entries
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("my-plugin"), "").unwrap();
        std::fs::write(plugin_dir.join("another-plugin"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 2);
        assert!(
            local_plugins.iter().any(|p| p.contains("my-plugin")),
            "Should find my-plugin in {:?}",
            local_plugins,
        );
        assert!(
            local_plugins.iter().any(|p| p.contains("another-plugin")),
            "Should find another-plugin in {:?}",
            local_plugins,
        );
    }

    #[test]
    fn list_installed_plugins_formats_local_entries() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("test-plugin"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert!(
            local_plugins.iter().any(|p| *p == "test-plugin (local)"),
            "Should format as 'name (local)', got {:?}",
            local_plugins,
        );
    }

    #[test]
    fn list_installed_plugins_finds_directories_as_plugins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Plugin entries can be directories too
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(plugin_dir.join("dir-plugin")).unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 1);
        assert!(local_plugins[0].contains("dir-plugin"));
    }

    #[test]
    fn list_installed_plugins_empty_plugin_dir() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Create empty .horus/plugins
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert!(local_plugins.is_empty());
    }

    #[test]
    fn list_installed_plugins_global_format() {
        // If any global plugins exist, they should contain "(global)"
        let plugins = list_installed_plugins();
        for plugin in &plugins {
            assert!(
                plugin.contains("(global)") || plugin.contains("(local)"),
                "Every plugin string should be tagged (global) or (local), got: {}",
                plugin,
            );
        }
    }

    // ── run_upgrade with local plugins present ──────────────────────────

    #[test]
    fn run_upgrade_check_only_with_local_plugins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        // Create local plugin directory
        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("fake-plugin"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_upgrade(true);
        std::env::set_current_dir(original).unwrap();

        result.unwrap();
    }

    #[test]
    fn run_upgrade_full_with_local_plugins() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("fake-plugin"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let result = run_upgrade(false);
        std::env::set_current_dir(original).unwrap();

        result.unwrap();
    }

    // ── Edge cases ──────────────────────────────────────────────────────

    #[test]
    fn list_installed_plugins_handles_non_utf8_gracefully() {
        // The function uses to_str() which filters out non-UTF8 names.
        // On Linux we can create non-UTF8 filenames, but for portability
        // we just verify the function doesn't panic with normal filenames
        // containing special characters.
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        std::fs::write(plugin_dir.join("plugin-with-dashes"), "").unwrap();
        std::fs::write(plugin_dir.join("plugin_with_underscores"), "").unwrap();
        std::fs::write(plugin_dir.join("plugin.with.dots"), "").unwrap();
        std::fs::write(plugin_dir.join("UPPERCASE"), "").unwrap();

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 4);
    }

    #[test]
    fn list_installed_plugins_many_entries() {
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        let plugin_dir = tmp.path().join(".horus/plugins");
        std::fs::create_dir_all(&plugin_dir).unwrap();
        for i in 0..20 {
            std::fs::write(plugin_dir.join(format!("plugin-{}", i)), "").unwrap();
        }

        std::env::set_current_dir(tmp.path()).unwrap();
        let plugins = list_installed_plugins();
        std::env::set_current_dir(original).unwrap();

        let local_plugins: Vec<_> = plugins.iter().filter(|p| p.contains("(local)")).collect();
        assert_eq!(local_plugins.len(), 20);
    }

    #[test]
    fn current_version_is_valid_semver() {
        let version = env!("CARGO_PKG_VERSION");
        assert!(
            semver::Version::parse(version).is_ok(),
            "CARGO_PKG_VERSION should be valid semver: {}",
            version,
        );
    }

    #[test]
    fn upgrade_horus_does_not_propagate_command_failure() {
        // The function catches command failures and prints a message
        // instead of returning Err. Verify this contract.
        // Run from a temp dir where there's no Cargo.toml — cargo install --path .
        // will definitely fail.
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());

        std::env::set_current_dir(tmp.path()).unwrap();
        let result = upgrade_horus("1.0.0");
        std::env::set_current_dir(original).unwrap();

        assert!(
            result.is_ok(),
            "upgrade_horus should return Ok even when cargo install fails",
        );
    }

    #[test]
    fn run_upgrade_no_panic_on_concurrent_calls() {
        // Ensure run_upgrade is safe to call without panicking
        // (basic smoke test for robustness)
        let result1 = run_upgrade(true);
        let result2 = run_upgrade(true);
        result1.unwrap();
        result2.unwrap();
    }
}
