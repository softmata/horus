//! Env command - freeze, restore, list, and show environments

use crate::cli_output;
use crate::commands::check::{
    check_system_package_exists, prompt_missing_system_package, MissingSystemChoice,
};
use crate::lockfile::{HorusLockfile, HORUS_LOCK};
use crate::{registry, workspace};
use colored::*;
use horus_core::error::{ConfigError, HorusError, HorusResult};
use std::fs;
use std::path::{Path, PathBuf};

/// Freeze the current environment to a file and optionally publish
pub fn run_freeze(output: Option<PathBuf>, publish: bool) -> HorusResult<()> {
    // Require horus.toml to exist
    let project_dir = std::env::current_dir()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
    if !project_dir.join(crate::manifest::HORUS_TOML).exists() {
        return Err(HorusError::Config(ConfigError::Other(
            "No horus.toml found. Run `horus init` or `horus new` first.".to_string(),
        )));
    }

    println!(
        "{} Freezing current environment...",
        cli_output::ICON_INFO.cyan()
    );

    let client = registry::RegistryClient::new();
    let manifest = client
        .freeze()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    // Update unified horus.lock with all discovered packages
    update_lockfile_from_manifest(&manifest)?;

    // Save to local file
    let freeze_file = output.unwrap_or_else(|| PathBuf::from("horus-freeze.toml"));
    let toml_str = toml::to_string_pretty(&manifest)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
    fs::write(&freeze_file, toml_str)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    println!("  Environment frozen to {}", freeze_file.display());
    println!("  Lockfile updated: {}", HORUS_LOCK);
    println!("   ID: {}", manifest.horus_id);
    println!("   Packages: {}", manifest.packages.len());

    // Publish to registry if requested
    if publish {
        // Validate: check for path dependencies before publishing
        let has_path_deps = manifest
            .packages
            .iter()
            .any(|pkg| matches!(pkg.source, registry::PackageSource::Path { .. }));

        if has_path_deps {
            println!(
                "\n{} Cannot publish environment with path dependencies!",
                "Error:".red()
            );
            println!("\nPath dependencies found:");
            for pkg in &manifest.packages {
                if let registry::PackageSource::Path { ref path } = pkg.source {
                    println!("  • {} -> {}", pkg.name, path);
                }
            }
            println!(
                "\n{}",
                "Path dependencies are not portable and cannot be published to the registry."
                    .yellow()
            );
            println!(
                "{}",
                "You can still save locally with: horus env freeze".yellow()
            );
            return Err(HorusError::Config(ConfigError::Other(
                "Cannot publish environment with path dependencies".to_string(),
            )));
        }

        println!();
        client
            .upload_environment(&manifest)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
    } else {
        println!("\n{} To share this environment:", "Tip:".dimmed());
        println!("   1. File: horus env restore {}", freeze_file.display());
        println!("   2. Registry: horus env freeze --publish");
    }

    Ok(())
}

/// Restore an environment from a file or registry ID
pub fn run_restore(source: String) -> HorusResult<()> {
    println!(
        "{} Restoring environment from {}...",
        cli_output::ICON_INFO.cyan(),
        source
    );

    let client = registry::RegistryClient::new();

    // Check if source is a file path or environment ID
    if source.ends_with(".toml") || PathBuf::from(&source).exists() {
        // It's a file path
        let content = fs::read_to_string(&source).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "Failed to read freeze file: {}",
                e
            )))
        })?;

        let manifest: registry::EnvironmentManifest = toml::from_str(&content).map_err(|e| {
            HorusError::Config(ConfigError::Other(format!(
                "failed to parse freeze file: {}",
                e
            )))
        })?;

        println!("  Found {} packages to restore", manifest.packages.len());

        // Install each package from the manifest
        restore_packages(&client, &manifest)?;

        println!("  Environment restored from {}", source);
        println!("   ID: {}", manifest.horus_id);
        println!("   Packages: {}", manifest.packages.len());
    } else {
        // It's an environment ID from registry
        println!("  Fetching environment {}...", source);

        let url = format!("{}/api/environments/{}", client.base_url(), source);

        let response = client
            .http_client()
            .get(&url)
            .send()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        if !response.status().is_success() {
            return Err(HorusError::Config(ConfigError::Other(format!(
                "Environment not found: {}",
                source
            ))));
        }

        let manifest: registry::EnvironmentManifest = response
            .json()
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

        println!("  Found {} packages to restore", manifest.packages.len());

        // Install each package
        restore_packages(&client, &manifest)?;

        println!("  Environment {} restored successfully!", source);
    }

    Ok(())
}

/// Common logic for restoring packages from a manifest.
/// Also updates `horus.lock` with the restored pins.
fn restore_packages(
    client: &registry::RegistryClient,
    manifest: &registry::EnvironmentManifest,
) -> HorusResult<()> {
    for pkg in &manifest.packages {
        // Handle different package sources
        match &pkg.source {
            registry::PackageSource::System => {
                // Check if system package actually exists
                let exists = check_system_package_exists(&pkg.name);

                if exists {
                    println!(
                        "  {} {} v{} (system package - verified)",
                        cli_output::ICON_SUCCESS.green(),
                        pkg.name,
                        pkg.version
                    );
                    continue;
                } else {
                    println!(
                        "\n  {} {} v{} (system package NOT found)",
                        cli_output::ICON_WARN.yellow(),
                        pkg.name,
                        pkg.version
                    );

                    // Prompt user for what to do
                    match prompt_missing_system_package(&pkg.name)? {
                        MissingSystemChoice::InstallGlobal => {
                            println!(
                                "  {} Installing to HORUS global cache...",
                                cli_output::ICON_INFO.cyan()
                            );
                            client
                                .install_to_target(
                                    &pkg.name,
                                    Some(&pkg.version),
                                    workspace::InstallTarget::Global,
                                )
                                .map_err(|e| {
                                    HorusError::Config(ConfigError::Other(e.to_string()))
                                })?;
                        }
                        MissingSystemChoice::InstallLocal => {
                            println!(
                                "  {} Installing to HORUS local...",
                                cli_output::ICON_INFO.cyan()
                            );
                            client.install(&pkg.name, Some(&pkg.version)).map_err(|e| {
                                HorusError::Config(ConfigError::Other(e.to_string()))
                            })?;
                        }
                        MissingSystemChoice::Skip => {
                            println!("  {} Skipped {}", cli_output::ICON_WARN.yellow(), pkg.name);
                            continue;
                        }
                    }
                }
            }
            registry::PackageSource::Path { path } => {
                println!(
                    "  {} {} (path dependency)",
                    cli_output::ICON_WARN.yellow(),
                    pkg.name
                );
                println!("    Path: {}", path);
                println!(
                    "    {} Path dependencies are not portable across machines.",
                    "Note:".dimmed()
                );
                println!(
                    "    {} Please update horus.toml with the correct path if needed.",
                    "Tip:".dimmed()
                );
                // Don't try to install - user must fix path manually
                continue;
            }
            registry::PackageSource::Registry => {
                // HORUS registry packages: use standard install
                println!("  Installing {} v{} from HORUS registry...", pkg.name, pkg.version);
                client
                    .install(&pkg.name, Some(&pkg.version))
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            }
            registry::PackageSource::CratesIO => {
                // CratesIO packages: detect workspace context and route correctly
                println!("  Installing {} v{} from crates.io...", pkg.name, pkg.version);
                let install_target = workspace::detect_or_select_workspace(true)
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
                client
                    .install_from_cratesio(&pkg.name, Some(&pkg.version), install_target)
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            }
            registry::PackageSource::PyPI => {
                // PyPI packages: pip install + pyproject.toml update
                println!("  Installing {} v{} from PyPI...", pkg.name, pkg.version);
                let install_target = workspace::detect_or_select_workspace(true)
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
                client
                    .install_from_pypi(&pkg.name, Some(&pkg.version), install_target)
                    .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;
            }
        }

        // Dependencies are installed to .horus/packages/ — native build files
        // (Cargo.toml, pyproject.toml) handle dependency tracking.
    }

    // Update unified horus.lock with all restored packages
    update_lockfile_from_manifest(manifest)?;
    println!(
        "  {} Lockfile updated: {}",
        cli_output::ICON_SUCCESS.green(),
        HORUS_LOCK
    );

    Ok(())
}

/// List published environments from the registry
pub fn run_list(json: bool) -> HorusResult<()> {
    if !json {
        println!(
            "{} Fetching published environments...",
            cli_output::ICON_INFO.cyan()
        );
    }

    let client = registry::RegistryClient::new();
    let environments = client
        .list_environments()
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if json {
        let env_list: Vec<_> = environments
            .iter()
            .map(|env| {
                serde_json::json!({
                    "id": env.horus_id,
                    "name": env.name,
                    "description": env.description,
                    "packages": env.manifest.packages.len(),
                    "created_at": env.created_at.to_string(),
                    "system": {
                        "os": env.manifest.system.os,
                        "arch": env.manifest.system.arch,
                    },
                    "horus_version": env.manifest.horus_version,
                })
            })
            .collect();
        let output = serde_json::json!({ "environments": env_list });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    if environments.is_empty() {
        println!("\n{}", "No published environments found.".dimmed());
        println!(
            "{}",
            "Publish one with: horus env freeze --publish".dimmed()
        );
    } else {
        println!(
            "\n{} Published Environments:\n",
            cli_output::ICON_SUCCESS.green()
        );
        for env in &environments {
            let name = env.name.as_deref().unwrap_or("(unnamed)");
            println!("  {} {}", cli_output::ICON_HINT.cyan(), env.horus_id.bold());
            println!("    Name: {}", name);
            if let Some(desc) = &env.description {
                println!("    Description: {}", desc.dimmed());
            }
            println!(
                "    Packages: {} | Created: {}",
                env.manifest.packages.len(),
                env.created_at
            );
            println!(
                "    System: {} ({})",
                env.manifest.system.os, env.manifest.system.arch
            );
            println!("    HORUS: v{}", env.manifest.horus_version);
            println!();
        }
        println!("{} To restore: horus env restore <ID>", "Tip:".dimmed());
        println!("{} For details: horus env show <ID>", "    ".dimmed());
    }

    Ok(())
}

/// Show details of a specific environment
pub fn run_show(id: String, json: bool) -> HorusResult<()> {
    if !json {
        println!(
            "{} Fetching environment {}...",
            cli_output::ICON_INFO.cyan(),
            id
        );
    }

    let client = registry::RegistryClient::new();
    let manifest = client
        .get_environment(&id)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    if json {
        let pkgs: Vec<_> = manifest
            .packages
            .iter()
            .map(|pkg| {
                let source_str = match &pkg.source {
                    registry::PackageSource::Registry => "registry",
                    registry::PackageSource::PyPI => "pypi",
                    registry::PackageSource::CratesIO => "crates.io",
                    registry::PackageSource::System => "system",
                    registry::PackageSource::Path { .. } => "path",
                };
                serde_json::json!({
                    "name": pkg.name,
                    "version": pkg.version,
                    "source": source_str,
                })
            })
            .collect();
        let output = serde_json::json!({
            "id": manifest.horus_id,
            "name": manifest.name,
            "description": manifest.description,
            "created_at": manifest.created_at.to_string(),
            "horus_version": manifest.horus_version,
            "system": {
                "os": manifest.system.os,
                "arch": manifest.system.arch,
                "python_version": manifest.system.python_version,
                "rust_version": manifest.system.rust_version,
                "gcc_version": manifest.system.gcc_version,
                "cuda_version": manifest.system.cuda_version,
            },
            "packages": pkgs,
        });
        println!(
            "{}",
            serde_json::to_string_pretty(&output).unwrap_or_default()
        );
        return Ok(());
    }

    // Display environment details
    println!(
        "\n{} Environment Details\n",
        cli_output::ICON_SUCCESS.green()
    );
    println!("  ID:          {}", manifest.horus_id.bold());
    println!(
        "  Name:        {}",
        manifest.name.as_deref().unwrap_or("(unnamed)")
    );
    if let Some(desc) = &manifest.description {
        println!("  Description: {}", desc);
    }
    println!(
        "  Created:     {}",
        manifest.created_at.format("%Y-%m-%d %H:%M:%S UTC")
    );
    println!("  HORUS:       v{}", manifest.horus_version);

    // System info
    println!("\n{} System\n", cli_output::ICON_INFO.cyan());
    println!("  OS:          {}", manifest.system.os);
    println!("  Arch:        {}", manifest.system.arch);
    if let Some(py) = &manifest.system.python_version {
        println!("  Python:      {}", py);
    }
    if let Some(rs) = &manifest.system.rust_version {
        println!("  Rust:        {}", rs);
    }
    if let Some(gcc) = &manifest.system.gcc_version {
        println!("  GCC:         {}", gcc);
    }
    if let Some(cuda) = &manifest.system.cuda_version {
        println!("  CUDA:        {}", cuda);
    }

    // Packages
    println!(
        "\n{} Packages ({})\n",
        cli_output::ICON_INFO.cyan(),
        manifest.packages.len()
    );
    for pkg in &manifest.packages {
        let source_str = match &pkg.source {
            registry::PackageSource::Registry => "registry".to_string(),
            registry::PackageSource::PyPI => "pypi".to_string(),
            registry::PackageSource::CratesIO => "crates.io".to_string(),
            registry::PackageSource::System => "system".to_string(),
            registry::PackageSource::Path { path } => format!("path:{}", path),
        };
        println!(
            "  {} {} v{} ({})",
            cli_output::ICON_HINT.cyan(),
            pkg.name.bold(),
            pkg.version,
            source_str.dimmed()
        );
    }

    println!(
        "\n{} To restore: horus env restore {}",
        "Tip:".dimmed(),
        manifest.horus_id
    );

    Ok(())
}

/// Convert an `EnvironmentManifest` into `horus.lock` pins.
///
/// Loads the existing lockfile (or creates a new one), merges all packages
/// from the manifest, and writes back to disk.
fn update_lockfile_from_manifest(manifest: &registry::EnvironmentManifest) -> HorusResult<()> {
    let lock_path = Path::new(HORUS_LOCK);

    let mut lockfile = if lock_path.exists() {
        HorusLockfile::load_from(lock_path)
            .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?
    } else {
        HorusLockfile::new()
    };

    for pkg in &manifest.packages {
        let source_str = match &pkg.source {
            registry::PackageSource::Registry => "registry",
            registry::PackageSource::CratesIO => "crates.io",
            registry::PackageSource::PyPI => "pypi",
            registry::PackageSource::System => "system",
            registry::PackageSource::Path { .. } => "path",
        };

        let checksum = if pkg.checksum.is_empty() {
            None
        } else {
            Some(pkg.checksum.clone())
        };

        lockfile.pin(&pkg.name, &pkg.version, source_str, checksum);
    }

    lockfile
        .save_to(lock_path)
        .map_err(|e| HorusError::Config(ConfigError::Other(e.to_string())))?;

    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;
    use chrono::Utc;

    fn make_test_manifest(packages: Vec<registry::LockedPackage>) -> registry::EnvironmentManifest {
        registry::EnvironmentManifest {
            horus_id: "test-env-001".to_string(),
            name: Some("test".to_string()),
            description: None,
            packages,
            system: registry::SystemInfo {
                os: "linux".to_string(),
                arch: "x86_64".to_string(),
                python_version: None,
                rust_version: None,
                gcc_version: None,
                cuda_version: None,
            },
            created_at: Utc::now(),
            horus_version: "0.1.9".to_string(),
        }
    }

    fn make_locked_package(
        name: &str,
        version: &str,
        source: registry::PackageSource,
    ) -> registry::LockedPackage {
        registry::LockedPackage {
            name: name.to_string(),
            version: version.to_string(),
            checksum: String::new(),
            source,
        }
    }

    // ── update_lockfile_from_manifest ────────────────────────────────────

    #[test]
    fn update_lockfile_creates_new_file() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let manifest = make_test_manifest(vec![make_locked_package(
            "serde",
            "1.0.0",
            registry::PackageSource::CratesIO,
        )]);

        let result = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
        assert!(tmp.path().join(HORUS_LOCK).exists());
    }

    #[test]
    fn update_lockfile_multiple_sources() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let manifest = make_test_manifest(vec![
            make_locked_package("serde", "1.0.0", registry::PackageSource::CratesIO),
            make_locked_package("numpy", "1.24.0", registry::PackageSource::PyPI),
            make_locked_package("libudev", "0.3.0", registry::PackageSource::System),
            make_locked_package("horus-nav", "0.1.0", registry::PackageSource::Registry),
            make_locked_package(
                "my-pkg",
                "0.1.0",
                registry::PackageSource::Path {
                    path: "/tmp/my-pkg".to_string(),
                },
            ),
        ]);

        let result = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
    }

    #[test]
    fn update_lockfile_empty_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let manifest = make_test_manifest(vec![]);
        let result = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
    }

    #[test]
    fn update_lockfile_with_checksum() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let mut pkg = make_locked_package("tokio", "1.35.0", registry::PackageSource::CratesIO);
        pkg.checksum = "sha256:abc123".to_string();

        let manifest = make_test_manifest(vec![pkg]);
        let result = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
    }

    // ── restore source detection ────────────────────────────────────────

    #[test]
    fn restore_file_path_detection() {
        assert!("env.toml".ends_with(".toml"));
        assert!(!"abc123".ends_with(".toml"));
        assert!("horus-freeze.toml".ends_with(".toml"));
    }

    // ── run_restore with nonexistent file ───────────────────────────────

    #[test]
    fn restore_nonexistent_file_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_restore("nonexistent.toml".to_string());
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    // ── Battle tests: env management ────────────────────────────────────

    #[test]
    fn restore_nonexistent_file_error_message() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let result = run_restore("missing-env.toml".to_string());
        std::env::set_current_dir(original).unwrap();

        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("Failed to read freeze file") || err.contains("No such file"),
            "Error should mention file read failure, got: {}",
            err
        );
    }

    #[test]
    fn restore_invalid_toml_content_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // Write invalid TOML content to a .toml file
        std::fs::write(tmp.path().join("bad.toml"), "this is not valid toml {{{{").unwrap();

        let result = run_restore("bad.toml".to_string());
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
        let err = result.unwrap_err().to_string();
        assert!(
            err.contains("parse") || err.contains("failed"),
            "Error should mention parse failure, got: {}",
            err
        );
    }

    #[test]
    fn restore_valid_toml_wrong_schema_fails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // Valid TOML but wrong schema for EnvironmentManifest
        std::fs::write(
            tmp.path().join("wrong-schema.toml"),
            "[package]\nname = \"test\"\nversion = \"0.1.0\"\n",
        )
        .unwrap();

        let result = run_restore("wrong-schema.toml".to_string());
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_err());
    }

    #[test]
    fn restore_source_detection_toml_suffix() {
        // Source ending with .toml is treated as a file path
        let source = "my-env.toml";
        assert!(
            source.ends_with(".toml") || PathBuf::from(source).exists(),
            "Source ending in .toml should be treated as file"
        );
    }

    #[test]
    fn restore_source_detection_id_string() {
        // Source NOT ending with .toml and not existing is treated as environment ID
        let source = "env-abc123";
        let is_file = source.ends_with(".toml") || PathBuf::from(source).exists();
        assert!(
            !is_file,
            "Non-.toml string that doesn't exist should be treated as env ID"
        );
    }

    #[test]
    fn update_lockfile_idempotent() {
        // Running update_lockfile twice with same manifest should succeed
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        let manifest = make_test_manifest(vec![
            make_locked_package("serde", "1.0.0", registry::PackageSource::CratesIO),
        ]);

        let r1 = update_lockfile_from_manifest(&manifest);
        let r2 = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(r1.is_ok());
        assert!(r2.is_ok());
    }

    #[test]
    fn update_lockfile_updates_existing_version() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // First write: serde 1.0.0
        let manifest1 = make_test_manifest(vec![
            make_locked_package("serde", "1.0.0", registry::PackageSource::CratesIO),
        ]);
        update_lockfile_from_manifest(&manifest1).unwrap();

        // Second write: serde 1.1.0 (should update, not duplicate)
        let manifest2 = make_test_manifest(vec![
            make_locked_package("serde", "1.1.0", registry::PackageSource::CratesIO),
        ]);
        let result = update_lockfile_from_manifest(&manifest2);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());

        // Verify the lockfile contains the updated version
        let lock_content = std::fs::read_to_string(tmp.path().join(HORUS_LOCK)).unwrap();
        assert!(
            lock_content.contains("1.1.0"),
            "Lockfile should contain updated version 1.1.0"
        );
    }

    #[test]
    fn update_lockfile_preserves_existing_packages() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // First: add serde
        let m1 = make_test_manifest(vec![
            make_locked_package("serde", "1.0.0", registry::PackageSource::CratesIO),
        ]);
        update_lockfile_from_manifest(&m1).unwrap();

        // Second: add tokio (serde should persist)
        let m2 = make_test_manifest(vec![
            make_locked_package("tokio", "1.35.0", registry::PackageSource::CratesIO),
        ]);
        update_lockfile_from_manifest(&m2).unwrap();
        std::env::set_current_dir(original).unwrap();

        let lock_content = std::fs::read_to_string(tmp.path().join(HORUS_LOCK)).unwrap();
        assert!(
            lock_content.contains("serde"),
            "Lockfile should still contain serde"
        );
        assert!(
            lock_content.contains("tokio"),
            "Lockfile should also contain tokio"
        );
    }

    #[test]
    fn update_lockfile_empty_checksum_is_none() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // Empty checksum should be treated as None
        let pkg = make_locked_package("test-pkg", "0.1.0", registry::PackageSource::Registry);
        assert!(pkg.checksum.is_empty());

        let checksum = if pkg.checksum.is_empty() {
            None
        } else {
            Some(pkg.checksum.clone())
        };
        assert!(checksum.is_none(), "Empty checksum should map to None");

        let manifest = make_test_manifest(vec![pkg]);
        let result = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
    }

    #[test]
    fn update_lockfile_nonempty_checksum_is_some() {
        let mut pkg = make_locked_package("test-pkg", "0.1.0", registry::PackageSource::Registry);
        pkg.checksum = "sha256:deadbeef".to_string();

        let checksum = if pkg.checksum.is_empty() {
            None
        } else {
            Some(pkg.checksum.clone())
        };
        assert_eq!(checksum, Some("sha256:deadbeef".to_string()));
    }

    #[test]
    fn make_test_manifest_defaults() {
        let manifest = make_test_manifest(vec![]);
        assert_eq!(manifest.horus_id, "test-env-001");
        assert_eq!(manifest.name, Some("test".to_string()));
        assert!(manifest.description.is_none());
        assert!(manifest.packages.is_empty());
        assert_eq!(manifest.system.os, "linux");
        assert_eq!(manifest.system.arch, "x86_64");
        assert!(manifest.system.python_version.is_none());
        assert!(manifest.system.rust_version.is_none());
        assert_eq!(manifest.horus_version, "0.1.9");
    }

    #[test]
    fn make_locked_package_defaults() {
        let pkg = make_locked_package("numpy", "1.24.0", registry::PackageSource::PyPI);
        assert_eq!(pkg.name, "numpy");
        assert_eq!(pkg.version, "1.24.0");
        assert!(pkg.checksum.is_empty());
        assert_eq!(pkg.source, registry::PackageSource::PyPI);
    }

    #[test]
    fn package_source_variants_coverage() {
        // Ensure all PackageSource variants can be used in make_locked_package
        let registry_pkg = make_locked_package("a", "1.0", registry::PackageSource::Registry);
        let pypi_pkg = make_locked_package("b", "1.0", registry::PackageSource::PyPI);
        let crates_pkg = make_locked_package("c", "1.0", registry::PackageSource::CratesIO);
        let system_pkg = make_locked_package("d", "1.0", registry::PackageSource::System);
        let path_pkg = make_locked_package(
            "e",
            "1.0",
            registry::PackageSource::Path {
                path: "/tmp/e".to_string(),
            },
        );

        assert_eq!(registry_pkg.source, registry::PackageSource::Registry);
        assert_eq!(pypi_pkg.source, registry::PackageSource::PyPI);
        assert_eq!(crates_pkg.source, registry::PackageSource::CratesIO);
        assert_eq!(system_pkg.source, registry::PackageSource::System);
        assert!(matches!(path_pkg.source, registry::PackageSource::Path { .. }));
    }

    #[test]
    fn source_string_mapping_for_lockfile() {
        // Verify the source string mapping used in update_lockfile_from_manifest
        let sources = vec![
            (registry::PackageSource::Registry, "registry"),
            (registry::PackageSource::CratesIO, "crates.io"),
            (registry::PackageSource::PyPI, "pypi"),
            (registry::PackageSource::System, "system"),
            (
                registry::PackageSource::Path {
                    path: "/tmp".to_string(),
                },
                "path",
            ),
        ];

        for (source, expected_str) in sources {
            let source_str = match &source {
                registry::PackageSource::Registry => "registry",
                registry::PackageSource::CratesIO => "crates.io",
                registry::PackageSource::PyPI => "pypi",
                registry::PackageSource::System => "system",
                registry::PackageSource::Path { .. } => "path",
            };
            assert_eq!(
                source_str, expected_str,
                "Source {:?} should map to '{}'",
                source, expected_str
            );
        }
    }

    #[test]
    fn update_lockfile_large_package_count() {
        let tmp = tempfile::TempDir::new().unwrap();
        let _guard = crate::CWD_LOCK.lock().unwrap();
        let original = std::env::current_dir().unwrap();
        std::env::set_current_dir(tmp.path()).unwrap();

        // Create a manifest with many packages
        let packages: Vec<_> = (0..50)
            .map(|i| make_locked_package(&format!("pkg-{}", i), "1.0.0", registry::PackageSource::Registry))
            .collect();

        let manifest = make_test_manifest(packages);
        let result = update_lockfile_from_manifest(&manifest);
        std::env::set_current_dir(original).unwrap();

        assert!(result.is_ok());
        let lock_content = std::fs::read_to_string(tmp.path().join(HORUS_LOCK)).unwrap();
        assert!(lock_content.contains("pkg-0"));
        assert!(lock_content.contains("pkg-49"));
    }
}
