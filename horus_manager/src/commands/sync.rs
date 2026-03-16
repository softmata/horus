//! `horus sync` — synchronize development environment.
//!
//! Checks/installs toolchains and system dependencies, writes horus.lock.

use anyhow::Result;
use colored::*;
use horus_sys::sync::{self, SyncManifest, SystemDep};

use crate::manifest::HorusManifest;

/// Implement SyncManifest for HorusManifest.
impl SyncManifest for HorusManifest {
    fn rust_edition(&self) -> Option<String> {
        Some(self.package.edition.clone())
    }

    fn python_version(&self) -> Option<String> {
        // Check if any Python dependencies exist
        let has_python = self
            .dependencies
            .iter()
            .any(|(_, dep)| {
                if let crate::manifest::DependencyValue::Detailed(d) = dep {
                    matches!(d.source, Some(crate::manifest::DepSource::PyPI))
                } else {
                    false
                }
            });
        if has_python {
            Some(">=3.9".to_string())
        } else {
            None
        }
    }

    fn system_deps(&self) -> Vec<SystemDep> {
        self.dependencies
            .iter()
            .filter_map(|(name, dep)| {
                if let crate::manifest::DependencyValue::Detailed(d) = dep {
                    if matches!(d.source, Some(crate::manifest::DepSource::System)) {
                        return Some(SystemDep {
                            name: name.clone(),
                            apt: d.apt.clone().or_else(|| Some(name.clone())),
                            brew: Some(name.clone()),
                            choco: Some(name.clone()),
                            pkg_config: d.cmake_package.clone(),
                        });
                    }
                }
                None
            })
            .collect()
    }

    fn needs_cpp(&self) -> bool {
        self.cpp.is_some()
    }

    fn project_name(&self) -> String {
        self.package.name.clone()
    }
}

/// Run `horus sync` — check and install all dependencies.
pub fn run_sync(check_only: bool) -> Result<()> {
    let (manifest, project_dir) = match HorusManifest::find_and_load() {
        Ok(m) => m,
        Err(_) => {
            println!(
                "{} No horus.toml found. Run {} to create a project.",
                "✗".red(),
                "horus new".cyan()
            );
            return Ok(());
        }
    };

    println!(
        "{} {} environment for {}...",
        "▸".cyan(),
        if check_only { "Checking" } else { "Syncing" },
        manifest.package.name.bold()
    );

    let report = if check_only {
        sync::check_environment(&manifest)
    } else {
        sync::sync_environment(&manifest)?
    };

    // Print results
    for item in &report.items {
        if item.installed {
            let version = item.version.as_deref().unwrap_or("installed");
            println!("  {} {} ({})", "✓".green(), item.name, version);
        } else {
            println!("  {} {} — not installed", "✗".red(), item.name);
            if let Some(ref cmd) = item.install_cmd {
                println!("    Install: {}", cmd.dimmed());
            }
        }
    }

    // Write lockfile if syncing (not just checking)
    if !check_only {
        let lockfile = sync::generate_lockfile(&manifest);
        let lock_path = project_dir.join("horus-sync.lock");
        lockfile.write(&lock_path)?;
        println!(
            "\n  {} Environment synced — {} updated",
            "✓".green(),
            "horus-sync.lock".bold()
        );
    }

    if !report.all_satisfied {
        println!(
            "\n{} Some dependencies are missing. Run {} to install.",
            "!".yellow(),
            "horus sync".cyan()
        );
    }

    Ok(())
}
