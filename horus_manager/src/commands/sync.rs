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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::manifest::{
        CppConfig, DepSource, DependencyValue, DetailedDependency, HooksConfig, IgnoreConfig,
        PackageInfo,
    };
    use std::collections::BTreeMap;

    fn base_manifest() -> HorusManifest {
        HorusManifest {
            package: PackageInfo {
                name: "my-robot".to_string(),
                version: "1.0.0".to_string(),
                description: None,
                authors: vec![],
                license: None,
                edition: "2024".to_string(),
                repository: None,
                package_type: None,
                categories: vec![],
                standard: None,
                rust_edition: None,
            },
            dependencies: BTreeMap::new(),
            dev_dependencies: BTreeMap::new(),
            drivers: BTreeMap::new(),
            scripts: BTreeMap::new(),
            ignore: IgnoreConfig::default(),
            enable: vec![],
            cpp: None,
            hooks: HooksConfig::default(),
        }
    }

    // ── rust_edition ─────────────────────────────────────────────────────

    #[test]
    fn rust_edition_returns_edition_from_manifest() {
        let m = base_manifest();
        assert_eq!(m.rust_edition(), Some("2024".to_string()));
    }

    #[test]
    fn rust_edition_returns_default_edition() {
        let mut m = base_manifest();
        m.package.edition = "1".to_string();
        assert_eq!(m.rust_edition(), Some("1".to_string()));
    }

    // ── python_version ───────────────────────────────────────────────────

    #[test]
    fn python_version_none_without_pypi_deps() {
        let m = base_manifest();
        assert_eq!(m.python_version(), None);
    }

    #[test]
    fn python_version_some_with_pypi_dep() {
        let mut m = base_manifest();
        m.dependencies.insert(
            "numpy".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some(">=1.24".to_string()),
                source: Some(DepSource::PyPI),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                apt: None,
                cmake_package: None,
            }),
        );
        let py = m.python_version();
        assert!(py.is_some());
        assert_eq!(py.unwrap(), ">=3.9");
    }

    #[test]
    fn python_version_none_with_crates_io_dep() {
        let mut m = base_manifest();
        m.dependencies.insert(
            "serde".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some("1.0".to_string()),
                source: Some(DepSource::CratesIo),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                apt: None,
                cmake_package: None,
            }),
        );
        assert_eq!(m.python_version(), None);
    }

    #[test]
    fn python_version_none_with_simple_dep() {
        let mut m = base_manifest();
        m.dependencies.insert(
            "horus_library".to_string(),
            DependencyValue::Simple("0.1.9".to_string()),
        );
        assert_eq!(m.python_version(), None);
    }

    // ── system_deps ──────────────────────────────────────────────────────

    #[test]
    fn system_deps_empty_without_system_deps() {
        let m = base_manifest();
        assert!(m.system_deps().is_empty());
    }

    #[test]
    fn system_deps_extracts_system_source_dep() {
        let mut m = base_manifest();
        m.dependencies.insert(
            "opencv".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: None,
                source: Some(DepSource::System),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                apt: Some("libopencv-dev".to_string()),
                cmake_package: Some("OpenCV".to_string()),
            }),
        );
        let deps = m.system_deps();
        assert_eq!(deps.len(), 1);
        assert_eq!(deps[0].name, "opencv");
        assert_eq!(deps[0].apt, Some("libopencv-dev".to_string()));
        assert_eq!(deps[0].pkg_config, Some("OpenCV".to_string()));
    }

    #[test]
    fn system_deps_defaults_apt_to_name() {
        let mut m = base_manifest();
        m.dependencies.insert(
            "libeigen3".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: None,
                source: Some(DepSource::System),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                apt: None, // no explicit apt name
                cmake_package: None,
            }),
        );
        let deps = m.system_deps();
        assert_eq!(deps.len(), 1);
        // apt defaults to the dependency name
        assert_eq!(deps[0].apt, Some("libeigen3".to_string()));
    }

    #[test]
    fn system_deps_skips_non_system_deps() {
        let mut m = base_manifest();
        m.dependencies.insert(
            "serde".to_string(),
            DependencyValue::Detailed(DetailedDependency {
                version: Some("1.0".to_string()),
                source: Some(DepSource::CratesIo),
                features: vec![],
                optional: false,
                path: None,
                git: None,
                apt: None,
                cmake_package: None,
            }),
        );
        m.dependencies.insert(
            "numpy".to_string(),
            DependencyValue::Simple("1.24".to_string()),
        );
        assert!(m.system_deps().is_empty());
    }

    // ── needs_cpp ────────────────────────────────────────────────────────

    #[test]
    fn needs_cpp_false_by_default() {
        let m = base_manifest();
        assert!(!m.needs_cpp());
    }

    #[test]
    fn needs_cpp_true_with_cpp_config() {
        let mut m = base_manifest();
        m.cpp = Some(CppConfig::default());
        assert!(m.needs_cpp());
    }

    // ── project_name ─────────────────────────────────────────────────────

    #[test]
    fn project_name_returns_package_name() {
        let m = base_manifest();
        assert_eq!(m.project_name(), "my-robot");
    }

    #[test]
    fn project_name_with_different_name() {
        let mut m = base_manifest();
        m.package.name = "warehouse-bot".to_string();
        assert_eq!(m.project_name(), "warehouse-bot");
    }
}
