//! `horus self update` — upgrade horus CLI and plugins.
//!
//! Checks for newer versions of horus and installed plugins,
//! downloads and installs updates.

use anyhow::{Context, Result};
use colored::*;

/// Run `horus self update`.
///
/// - `check_only`: If true, show available updates without installing.
pub fn run_upgrade(check_only: bool) -> Result<()> {
    let current_version = env!("CARGO_PKG_VERSION");

    // The banner names the command that runs this: `horus self update`. It
    // said "horus upgrade", which this CLI does not have.
    println!("{}", "horus self update".bold());
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

/// Upgrade horus by rebuilding from the HORUS source tree.
///
/// This used to run `cargo install --path .`, which resolves against the
/// process's current working directory. `horus self update` is a command
/// people naturally run from inside a project, so it built and installed
/// *that* package — running its `build.rs` and proc-macro dependencies as the
/// user, then dropping its binaries into `~/.cargo/bin`, which is on PATH
/// ahead of the real CLI — and printed "Upgraded to <version>" either way.
/// The artifact must come from the resolved HORUS source tree, never from
/// `std::env::current_dir()`, and the success line must be earned.
pub(crate) fn upgrade_horus(version: &str) -> Result<()> {
    // `version` is whatever the registry's JSON said. Since it is about to
    // reach a process argument and be reported to the user as installed,
    // require it to be plain semver first: a hostile response cannot then
    // smuggle a leading `-` through as a cargo flag.
    semver::Version::parse(version)
        .with_context(|| format!("registry reported a non-semver version {:?}", version))?;

    println!("  Building horus {}...", version.cyan());

    let src_root = crate::commands::run::find_horus_source_dir()
        .context("cannot locate the HORUS source tree to build from; set HORUS_SOURCE")?;
    // find_horus_source_dir() returns the workspace root, whose Cargo.toml is a
    // virtual manifest — `cargo install --path` needs the package directory.
    let manifest_dir = src_root.join("horus_manager");
    let manifest = manifest_dir.join("Cargo.toml");
    let manifest_text = std::fs::read_to_string(&manifest)
        .with_context(|| format!("reading {}", manifest.display()))?;
    anyhow::ensure!(
        manifest_text.contains("name = \"horus_manager\""),
        "{} is not the horus_manager package; refusing to install from it",
        manifest.display()
    );

    let manual_hint = format!(
        "cargo install --path {} --locked --force",
        manifest_dir.display()
    );

    // --locked so the build cannot silently pick up newer transitive deps;
    // --force because the same version may already be installed.
    let status = std::process::Command::new("cargo")
        .current_dir(&manifest_dir)
        .args(["install", "--path", ".", "--locked", "--force"])
        .status();

    match status {
        Ok(s) if s.success() => {
            // A cargo exit code of 0 is not proof that the horus on PATH is
            // the new one, so ask the installed binary what it is rather than
            // asserting the upgrade happened.
            match installed_horus_version() {
                Some(installed) if installed == version => {
                    println!("  {} Upgraded to {}", "*".green(), version.green());
                }
                Some(installed) => {
                    println!(
                        "  {} Install completed, but the installed horus reports {} (expected {})",
                        "!".yellow(),
                        installed.yellow(),
                        version.dimmed()
                    );
                }
                None => {
                    println!(
                        "  {} Install completed, but the installed horus could not be run to confirm its version",
                        "!".yellow()
                    );
                }
            }
        }
        _ => {
            println!(
                "  {} Auto-upgrade failed. Manual upgrade: {}",
                "!".yellow(),
                manual_hint.dimmed()
            );
        }
    }

    Ok(())
}

/// Ask the `horus` binary in the cargo bin directory — where `cargo install`
/// just wrote — which version it is. `None` when it cannot be found or run.
fn installed_horus_version() -> Option<String> {
    let bin_name = if cfg!(windows) { "horus.exe" } else { "horus" };
    let bin = cargo_bin_dir()?.join(bin_name);
    if !bin.exists() {
        return None;
    }
    let output = std::process::Command::new(&bin)
        .arg("--version")
        .output()
        .ok()?;
    if !output.status.success() {
        return None;
    }
    // clap prints "horus <version>".
    String::from_utf8_lossy(&output.stdout)
        .split_whitespace()
        .nth(1)
        .map(str::to_string)
}

/// The directory `cargo install` writes binaries to: `$CARGO_HOME/bin`,
/// falling back to `~/.cargo/bin`.
fn cargo_bin_dir() -> Option<std::path::PathBuf> {
    match std::env::var("CARGO_HOME") {
        Ok(home) if !home.is_empty() => Some(std::path::PathBuf::from(home).join("bin")),
        _ => dirs::home_dir().map(|h| h.join(".cargo").join("bin")),
    }
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

    /// Point HORUS_SOURCE at an isolated tree that carries the marker
    /// `find_horus_source_dir` looks for but no `horus_manager` package, run
    /// `f`, and restore the environment. Every upgrade_horus test needs this:
    /// the build now comes from the resolved source tree, so without it a test
    /// on a developer machine would resolve the real workspace and start a
    /// full release build of the CLI.
    fn with_isolated_horus_source<T>(f: impl FnOnce(&std::path::Path) -> T) -> T {
        // CWD_LOCK doubles as the serialization point for process-global env.
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let tmp = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(tmp.path().join("horus")).unwrap();
        std::fs::write(
            tmp.path().join("horus/Cargo.toml"),
            "[package]\nname = \"horus\"\n",
        )
        .unwrap();

        let old_val = std::env::var("HORUS_SOURCE").ok();
        std::env::set_var("HORUS_SOURCE", tmp.path());
        let result = f(tmp.path());
        match old_val {
            Some(v) => std::env::set_var("HORUS_SOURCE", v),
            None => std::env::remove_var("HORUS_SOURCE"),
        }
        result
    }

    #[test]
    fn upgrade_horus_refuses_a_source_tree_without_horus_manager() {
        // Inverted: this test used to chdir into an empty temp dir and assert
        // Ok, pinning the old contract that the upgrade builds whatever is in
        // the current directory. The build now comes from the resolved HORUS
        // source tree, and a tree with no horus_manager package is an error,
        // not a silent no-op.
        let result = with_isolated_horus_source(|_| upgrade_horus("99.99.99"));
        let err = result.expect_err("no horus_manager package should be fatal");
        assert!(
            err.to_string().contains("Cargo.toml"),
            "error should name the manifest it could not read: {}",
            err
        );
    }

    #[test]
    fn upgrade_horus_rejects_non_semver_version() {
        // Regression: `version` comes from registry JSON and is passed to
        // cargo, so it is validated before anything is executed. The empty
        // string used to reach the build step and return Ok.
        let result = upgrade_horus("");
        assert!(result.is_err(), "empty version must be rejected");

        let result = upgrade_horus("--path");
        assert!(
            result.is_err(),
            "a version that looks like a cargo flag must be rejected",
        );
    }

    #[test]
    fn upgrade_horus_does_not_depend_on_the_current_directory() {
        // Regression for the cwd build: standing in a valid Cargo package must
        // not make the upgrade build it. With HORUS_SOURCE isolated the call
        // fails on the missing horus_manager manifest no matter where we
        // stand, which is exactly the point — cwd is not consulted.
        let cwd_pkg = tempfile::tempdir().unwrap();
        std::fs::create_dir_all(cwd_pkg.path().join("src")).unwrap();
        std::fs::write(
            cwd_pkg.path().join("Cargo.toml"),
            "[package]\nname = \"totally-not-horus\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();
        std::fs::write(cwd_pkg.path().join("src/main.rs"), "fn main() {}").unwrap();

        let result = with_isolated_horus_source(|_| {
            let original = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
            std::env::set_current_dir(cwd_pkg.path()).unwrap();
            let result = upgrade_horus("0.2.0");
            std::env::set_current_dir(original).unwrap();
            result
        });

        assert!(
            result.is_err(),
            "upgrade must resolve the HORUS source tree, not the cwd package",
        );
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
    fn upgrade_horus_reports_an_unusable_source_tree() {
        // Inverted: this used to assert Ok because a failing `cargo install`
        // is only printed, not propagated. That is still true of the build
        // step, but the pre-flight checks that replaced the cwd build — semver
        // validation and locating a real horus_manager manifest — do return
        // Err, so a source tree that cannot be built from is now reported
        // rather than swallowed.
        let result = with_isolated_horus_source(|_| upgrade_horus("1.0.0"));
        assert!(
            result.is_err(),
            "an unusable HORUS source tree should be reported, not swallowed",
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
