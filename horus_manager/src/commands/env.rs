//! Env command - freeze, restore, list, and show environments

use crate::commands::check::{check_system_package_exists, prompt_missing_system_package, MissingSystemChoice};
use crate::{registry, workspace, yaml_utils};
use colored::*;
use horus_core::error::{HorusError, HorusResult};
use std::fs;
use std::path::PathBuf;

/// Freeze the current environment to a file and optionally publish
pub fn run_freeze(output: Option<PathBuf>, publish: bool) -> HorusResult<()> {
    println!("{} Freezing current environment...", "▶".cyan());

    let client = registry::RegistryClient::new();
    let manifest = client
        .freeze()
        .map_err(|e| HorusError::Config(e.to_string()))?;

    // Save to local file
    let freeze_file = output.unwrap_or_else(|| PathBuf::from("horus-freeze.yaml"));
    let yaml = serde_yaml::to_string(&manifest)
        .map_err(|e| HorusError::Config(e.to_string()))?;
    fs::write(&freeze_file, yaml)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    println!("  Environment frozen to {}", freeze_file.display());
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
            println!("\n{}", "Path dependencies are not portable and cannot be published to the registry.".yellow());
            println!(
                "{}",
                "You can still save locally with: horus env freeze".yellow()
            );
            return Err(HorusError::Config(
                "Cannot publish environment with path dependencies".to_string(),
            ));
        }

        println!();
        client
            .upload_environment(&manifest)
            .map_err(|e| HorusError::Config(e.to_string()))?;
    } else {
        println!("\n{} To share this environment:", "Tip:".dimmed());
        println!("   1. File: horus env restore {}", freeze_file.display());
        println!("   2. Registry: horus env freeze --publish");
    }

    Ok(())
}

/// Restore an environment from a file or registry ID
pub fn run_restore(source: String) -> HorusResult<()> {
    println!("{} Restoring environment from {}...", "▶".cyan(), source);

    let client = registry::RegistryClient::new();

    // Check if source is a file path or environment ID
    if source.ends_with(".yaml")
        || source.ends_with(".yml")
        || PathBuf::from(&source).exists()
    {
        // It's a file path
        let content = fs::read_to_string(&source).map_err(|e| {
            HorusError::Config(format!("Failed to read freeze file: {}", e))
        })?;

        let manifest: registry::EnvironmentManifest =
            serde_yaml::from_str(&content).map_err(|e| {
                HorusError::Config(format!("Failed to parse freeze file: {}", e))
            })?;

        println!("  Found {} packages to restore", manifest.packages.len());

        // Get workspace path for horus.yaml updates
        let workspace_path = workspace::find_workspace_root();

        // Install each package from the manifest
        restore_packages(&client, &manifest, &workspace_path)?;

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
            .map_err(|e| HorusError::Config(e.to_string()))?;

        if !response.status().is_success() {
            return Err(HorusError::Config(format!(
                "Environment not found: {}",
                source
            )));
        }

        let manifest: registry::EnvironmentManifest = response
            .json()
            .map_err(|e| HorusError::Config(e.to_string()))?;

        println!("  Found {} packages to restore", manifest.packages.len());

        // Get workspace path for horus.yaml updates
        let workspace_path = workspace::find_workspace_root();

        // Install each package
        restore_packages(&client, &manifest, &workspace_path)?;

        println!("  Environment {} restored successfully!", source);
    }

    Ok(())
}

/// Common logic for restoring packages from a manifest
fn restore_packages(
    client: &registry::RegistryClient,
    manifest: &registry::EnvironmentManifest,
    workspace_path: &Option<PathBuf>,
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
                        "✓".green(),
                        pkg.name,
                        pkg.version
                    );
                    continue;
                } else {
                    println!(
                        "\n  {} {} v{} (system package NOT found)",
                        "[WARNING]".yellow(),
                        pkg.name,
                        pkg.version
                    );

                    // Prompt user for what to do
                    match prompt_missing_system_package(&pkg.name)? {
                        MissingSystemChoice::InstallGlobal => {
                            println!(
                                "  {} Installing to HORUS global cache...",
                                "▶".cyan()
                            );
                            client
                                .install_to_target(
                                    &pkg.name,
                                    Some(&pkg.version),
                                    workspace::InstallTarget::Global,
                                )
                                .map_err(|e| {
                                    HorusError::Config(e.to_string())
                                })?;
                        }
                        MissingSystemChoice::InstallLocal => {
                            println!(
                                "  {} Installing to HORUS local...",
                                "▶".cyan()
                            );
                            client
                                .install(&pkg.name, Some(&pkg.version))
                                .map_err(|e| {
                                    HorusError::Config(e.to_string())
                                })?;
                        }
                        MissingSystemChoice::Skip => {
                            println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                            continue;
                        }
                    }
                }
            }
            registry::PackageSource::Path { path } => {
                println!(
                    "  {} {} (path dependency)",
                    "[WARNING]".yellow(),
                    pkg.name
                );
                println!("    Path: {}", path);
                println!("    {} Path dependencies are not portable across machines.", "Note:".dimmed());
                println!("    {} Please update horus.yaml with the correct path if needed.", "Tip:".dimmed());
                // Don't try to install - user must fix path manually
                continue;
            }
            _ => {
                // Registry, PyPI, CratesIO - use standard install
                println!("  Installing {} v{}...", pkg.name, pkg.version);
                client
                    .install(&pkg.name, Some(&pkg.version))
                    .map_err(|e| HorusError::Config(e.to_string()))?;
            }
        }

        // Update horus.yaml if in a workspace
        if let Some(ref ws_path) = workspace_path {
            let yaml_path = ws_path.join("horus.yaml");
            if yaml_path.exists() {
                if let Err(e) =
                    yaml_utils::add_dependency_to_horus_yaml(
                        &yaml_path,
                        &pkg.name,
                        &pkg.version,
                    )
                {
                    eprintln!(
                        "  {} Failed to update horus.yaml: {}",
                        "⚠".yellow(),
                        e
                    );
                }
            }
        }
    }
    Ok(())
}

/// List published environments from the registry
pub fn run_list() -> HorusResult<()> {
    println!("{} Fetching published environments...", "▶".cyan());

    let client = registry::RegistryClient::new();
    let environments = client
        .list_environments()
        .map_err(|e| HorusError::Config(e.to_string()))?;

    if environments.is_empty() {
        println!("\n{}", "No published environments found.".dimmed());
        println!(
            "{}",
            "Publish one with: horus env freeze --publish".dimmed()
        );
    } else {
        println!("\n{} Published Environments:\n", "✓".green());
        for env in &environments {
            let name = env.name.as_deref().unwrap_or("(unnamed)");
            println!("  {} {}", "•".cyan(), env.horus_id.bold());
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
pub fn run_show(id: String) -> HorusResult<()> {
    println!("{} Fetching environment {}...", "▶".cyan(), id);

    let client = registry::RegistryClient::new();
    let manifest = client
        .get_environment(&id)
        .map_err(|e| HorusError::Config(e.to_string()))?;

    // Display environment details
    println!("\n{} Environment Details\n", "✓".green());
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
    println!("\n{} System\n", "▸".cyan());
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
    println!("\n{} Packages ({})\n", "▸".cyan(), manifest.packages.len());
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
            "•".cyan(),
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
