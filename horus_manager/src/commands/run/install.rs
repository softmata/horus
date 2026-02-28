use super::deps::{
    parse_horus_yaml_dependencies_v2, split_dependencies_with_context, CargoPackage, GitPackage,
    GitRef, PipPackage,
};
use crate::cargo_utils::detect_system_cargo_binary;
use crate::config::{CARGO_TOML, HORUS_YAML};
use crate::version;
use anyhow::{anyhow, bail, Context, Result};
use crate::cli_output;
use colored::*;
use std::collections::HashSet;
use std::fs;
use std::io::{self, Write};
#[cfg(unix)]
use std::os::unix::fs::symlink;
use std::path::{Path, PathBuf};
use std::process::Command;

pub(crate) fn clone_git_dependency(git_pkg: &GitPackage) -> Result<(String, PathBuf)> {
    let global_cache = home_dir().join(".horus/cache");
    let cache_dir_name = git_pkg.cache_dir_name();
    let cache_path = global_cache.join(&cache_dir_name);

    // Check if already cached
    if cache_path.exists() && cache_path.join(CARGO_TOML).exists() {
        println!(
            "  {} Git dependency '{}' cached at: {}",
            cli_output::ICON_SUCCESS.green(),
            git_pkg.name,
            cache_path.display()
        );
        return Ok((git_pkg.name.clone(), cache_path));
    }

    // Create cache directory
    fs::create_dir_all(&global_cache)?;

    // Clone the repository
    println!(
        "  {} Cloning git dependency: {} from {}",
        "↓".cyan(),
        git_pkg.name,
        git_pkg.url
    );

    // Remove stale directory if exists
    if cache_path.exists() {
        fs::remove_dir_all(&cache_path)?;
    }

    // Build git clone command
    let mut clone_cmd = Command::new("git");
    clone_cmd.args(["clone", "--depth", "1"]);

    // Add branch/tag/rev options
    match &git_pkg.git_ref {
        GitRef::Branch(branch) => {
            clone_cmd.args(["--branch", branch]);
        }
        GitRef::Tag(tag) => {
            clone_cmd.args(["--branch", tag]);
        }
        GitRef::Rev(_) => {
            // For specific rev, we need full clone (can't use --depth 1)
            clone_cmd.args(["--no-single-branch"]);
        }
        GitRef::Default => {}
    }

    clone_cmd.args([&git_pkg.url, &*cache_path.to_string_lossy()]);

    let output = clone_cmd.output().context("Failed to run git clone")?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(anyhow!("Git clone failed: {}", stderr));
    }

    // Checkout specific rev if specified
    if let GitRef::Rev(rev) = &git_pkg.git_ref {
        let checkout_output = Command::new("git")
            .args(["checkout", rev])
            .current_dir(&cache_path)
            .output()
            .context("Failed to checkout git revision")?;

        if !checkout_output.status.success() {
            let stderr = String::from_utf8_lossy(&checkout_output.stderr);
            return Err(anyhow!("Git checkout failed: {}", stderr));
        }
    }

    // Verify it's a valid Rust crate
    if !cache_path.join(CARGO_TOML).exists() {
        return Err(anyhow!(
            "Git dependency '{}' doesn't contain a Cargo.toml file",
            git_pkg.name
        ));
    }

    println!("  {} Cloned git dependency: {}", cli_output::ICON_SUCCESS.green(), git_pkg.name);

    Ok((git_pkg.name.clone(), cache_path))
}

/// Clone all git dependencies and return them as path dependencies
pub(crate) fn resolve_git_dependencies(git_deps: &[GitPackage]) -> Result<Vec<(String, PathBuf)>> {
    let mut resolved = Vec::new();

    for git_pkg in git_deps {
        match clone_git_dependency(git_pkg) {
            Ok((name, path)) => resolved.push((name, path)),
            Err(e) => {
                eprintln!(
                    "  {} Failed to clone git dependency '{}': {}",
                    cli_output::ICON_ERROR.red(),
                    git_pkg.name,
                    e
                );
                // Continue with other dependencies
            }
        }
    }

    Ok(resolved)
}

/// Install pip packages using global cache (HORUS philosophy)
/// Packages stored at: ~/.horus/cache/pypi_{name}@{version}/
pub(crate) fn install_pip_packages(packages: Vec<PipPackage>) -> Result<()> {
    if packages.is_empty() {
        return Ok(());
    }

    println!("{} Resolving Python packages...", "[PYTHON]".cyan());

    let global_cache = home_dir().join(".horus/cache");
    let local_packages = PathBuf::from(".horus/packages");

    fs::create_dir_all(&global_cache)?;
    fs::create_dir_all(&local_packages)?;

    // Use system Python's pip directly
    let python_cmd = super::detect_python_interpreter()?;

    for pkg in &packages {
        // Check if package exists in system first
        if let Ok(Some(system_version)) = detect_system_python_package(&pkg.name) {
            // Auto-use system package for horus-robotics (core dependency)
            if pkg.name == "horus-robotics" {
                create_system_reference_python_run(&pkg.name, &system_version)?;
                continue;
            }

            let local_link = local_packages.join(&pkg.name);

            // Skip if already handled
            if local_link.exists() || local_link.read_link().is_ok() {
                continue;
            }

            // Prompt user for choice
            match prompt_system_package_choice_run(&pkg.name, &system_version)? {
                SystemPackageChoiceRun::UseSystem => {
                    create_system_reference_python_run(&pkg.name, &system_version)?;
                    continue;
                }
                SystemPackageChoiceRun::InstallHORUS => {
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                    // Continue with installation below
                }
                SystemPackageChoiceRun::Cancel => {
                    println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                    continue;
                }
            }
        }

        // Get actual version by querying PyPI or using installed version
        let version_str = pkg
            .version
            .as_ref()
            .map(|v| {
                v.replace(">=", "")
                    .replace("==", "")
                    .replace("~=", "")
                    .replace(">", "")
                    .replace("<", "")
            })
            .unwrap_or_else(|| "latest".to_string());

        // Cache directory with pypi_ prefix to distinguish from HORUS packages
        let pkg_cache_dir = global_cache.join(format!("pypi_{}@{}", pkg.name, version_str));

        let local_link = local_packages.join(&pkg.name);

        // If already symlinked, skip
        if local_link.exists() || local_link.read_link().is_ok() {
            println!("  {} {} (already linked)", cli_output::ICON_SUCCESS.green(), pkg.name);
            continue;
        }

        // If not cached, install to global cache
        if !pkg_cache_dir.exists() {
            println!("  {} Installing {} to global cache...", cli_output::ICON_INFO.cyan(), pkg.name);

            fs::create_dir_all(&pkg_cache_dir)?;

            // Install package with pip to cache directory using system pip
            let mut cmd = Command::new(&python_cmd);
            cmd.args([
                "-m",
                "pip",
                "install",
                "--target",
                &*pkg_cache_dir.to_string_lossy(),
            ]);
            cmd.arg(pkg.requirement_string());

            let output = cmd.output().context("Failed to run pip install")?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                bail!("pip install failed for {}: {}", pkg.name, stderr);
            }

            // Create metadata.json for package tracking
            let metadata = serde_json::json!({
                "name": pkg.name,
                "version": version_str,
                "source": "PyPI"
            });
            let metadata_path = pkg_cache_dir.join("metadata.json");
            fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

            println!("  {} Cached {}", cli_output::ICON_SUCCESS.green(), pkg.name);
        } else {
            println!("  {} {} -> global cache", "↗".cyan(), pkg.name);
        }

        // Symlink from local packages to global cache
        symlink(&pkg_cache_dir, &local_link)
            .context(format!("Failed to symlink {} from global cache", pkg.name))?;
        println!("  {} Linked {}", cli_output::ICON_SUCCESS.green(), pkg.name);
    }

    Ok(())
}

pub(crate) fn install_cargo_packages(packages: Vec<CargoPackage>) -> Result<()> {
    if packages.is_empty() {
        return Ok(());
    }

    println!("{} Resolving Rust binaries...", "[RUST]".cyan());

    let global_cache = home_dir().join(".horus/cache");
    let local_bin = PathBuf::from(".horus/bin");
    let local_packages = PathBuf::from(".horus/packages");

    fs::create_dir_all(&global_cache)?;
    fs::create_dir_all(&local_bin)?;
    fs::create_dir_all(&local_packages)?;

    // Check if cargo is available
    if Command::new("cargo").arg("--version").output().is_err() {
        bail!("cargo not found. Please install Rust toolchain from https://rustup.rs");
    }

    for pkg in &packages {
        // Check if system binary exists first
        if let Ok(Some(system_version)) = detect_system_cargo_binary(&pkg.name) {
            let local_link = local_bin.join(&pkg.name);

            // Skip if already handled
            if local_link.exists() || local_link.read_link().is_ok() {
                continue;
            }

            // Prompt user for choice
            match prompt_system_cargo_choice_run(&pkg.name, &system_version)? {
                SystemPackageChoiceRun::UseSystem => {
                    create_system_reference_cargo_run(&pkg.name, &system_version)?;
                    continue;
                }
                SystemPackageChoiceRun::InstallHORUS => {
                    println!("  {} Installing isolated copy to HORUS...", "".blue());
                    // Continue with installation below
                }
                SystemPackageChoiceRun::Cancel => {
                    println!("  {} Skipped {}", "⊘".yellow(), pkg.name);
                    continue;
                }
            }
        }

        let version_str = pkg
            .version
            .as_ref()
            .unwrap_or(&"latest".to_string())
            .clone();
        let pkg_cache_dir = global_cache.join(format!("cratesio_{}@{}", pkg.name, version_str));
        let local_link = local_bin.join(&pkg.name);

        // If already linked, skip
        if local_link.exists() || local_link.read_link().is_ok() {
            println!("  {} {} (already linked)", cli_output::ICON_SUCCESS.green(), pkg.name);
            continue;
        }

        // If not cached, install to global cache
        if !pkg_cache_dir.exists() {
            println!("  {} Installing {} to global cache...", cli_output::ICON_INFO.cyan(), pkg.name);

            fs::create_dir_all(&pkg_cache_dir)?;

            // Install with cargo to cache directory
            let mut cmd = Command::new("cargo");
            cmd.arg("install");

            if let Some(version) = &pkg.version {
                cmd.arg(format!("{}@{}", pkg.name, version));
            } else {
                cmd.arg(&pkg.name);
            }

            cmd.arg("--root").arg(&pkg_cache_dir);

            let output = cmd.output().context("Failed to run cargo install")?;

            if !output.status.success() {
                let stderr = String::from_utf8_lossy(&output.stderr);
                bail!("cargo install failed for {}: {}", pkg.name, stderr);
            }

            // Create metadata.json for package tracking
            let metadata = serde_json::json!({
                "name": pkg.name,
                "version": version_str,
                "source": "CratesIO"
            });
            let metadata_path = pkg_cache_dir.join("metadata.json");
            fs::write(&metadata_path, serde_json::to_string_pretty(&metadata)?)?;

            println!("  {} Cached {}", cli_output::ICON_SUCCESS.green(), pkg.name);
        } else {
            println!("  {} {} -> global cache", "↗".cyan(), pkg.name);
        }

        // Symlink binary from cache/bin/ to .horus/bin/
        let cached_bin = pkg_cache_dir.join("bin").join(&pkg.name);
        if cached_bin.exists() {
            symlink(&cached_bin, &local_link)
                .context(format!("Failed to symlink {} from global cache", pkg.name))?;
            println!("  {} Linked {}", cli_output::ICON_SUCCESS.green(), pkg.name);
        } else {
            println!(
                "  {} Warning: Binary {} not found in cache",
                "[WARNING]".yellow(),
                pkg.name
            );
        }
    }

    Ok(())
}

pub(crate) fn resolve_dependencies(dependencies: HashSet<String>) -> Result<()> {
    resolve_dependencies_with_context(dependencies, None)
}

pub(crate) fn resolve_dependencies_with_context(
    dependencies: HashSet<String>,
    context_language: Option<&str>,
) -> Result<()> {
    // Check version compatibility first
    if let Err(e) = version::check_version_compatibility() {
        eprintln!("\n{}", "Hint:".cyan());
        eprintln!("  If you recently updated HORUS, run ./install.sh to update libraries.");
        return Err(e);
    }

    // Split dependencies into HORUS packages, pip packages, and cargo packages
    let (horus_packages, pip_packages, cargo_packages) =
        split_dependencies_with_context(dependencies, context_language);

    // Resolve HORUS packages (existing logic)
    if !horus_packages.is_empty() {
        resolve_horus_packages(horus_packages.into_iter().collect())?;
    }

    // Resolve pip packages
    if !pip_packages.is_empty() {
        install_pip_packages(pip_packages)?;
    }

    // Resolve cargo packages - skip for Python (library crates can't be installed with cargo install)
    // Cargo library dependencies are handled by Cargo.toml for Rust projects
    if !cargo_packages.is_empty() && context_language != Some("python") {
        install_cargo_packages(cargo_packages)?;
    }

    Ok(())
}

pub(crate) fn resolve_horus_packages(dependencies: HashSet<String>) -> Result<()> {
    let global_cache = home_dir().join(".horus/cache");
    let local_packages = PathBuf::from(".horus/packages");

    // Ensure directories exist
    fs::create_dir_all(&global_cache)?;
    fs::create_dir_all(&local_packages)?;

    // Collect missing packages first
    let mut missing_packages = Vec::new();

    for package in &dependencies {
        let local_link = local_packages.join(package);

        // Skip if already linked
        if local_link.exists() {
            println!("  {} {} (already linked)", cli_output::ICON_SUCCESS.green(), package);
            continue;
        }

        // Check global cache
        let cached_versions = find_cached_versions(&global_cache, package)?;

        if let Some(cached) = cached_versions.first() {
            // Check if we're using a different version than requested
            let cached_name = cached.file_name().and_then(|n| n.to_str()).unwrap_or("");
            let version_mismatch = package.contains('@') && cached_name != package;

            // Special handling for horus_py - the Python package is named "horus"
            if package.starts_with("horus_py") {
                // Check if lib/horus exists in the cached package
                let lib_horus = cached.join("lib/horus");
                if lib_horus.exists() {
                    // Create symlink named "horus" pointing to lib/horus
                    let horus_link = local_packages.join("horus");

                    // Check if symlink already exists
                    if horus_link.exists() {
                        println!("  {} {} (already linked)", cli_output::ICON_SUCCESS.green(), package);
                        continue;
                    }

                    if version_mismatch {
                        println!(
                            "  {} {} -> {} (using {})",
                            "↗".cyan(),
                            package,
                            "global cache".dimmed(),
                            cached_name.yellow()
                        );
                    } else {
                        println!(
                            "  {} {} -> {}",
                            "↗".cyan(),
                            package,
                            "global cache".dimmed()
                        );
                    }
                    symlink(&lib_horus, &horus_link).context("Failed to symlink horus_py")?;
                    continue;
                }
            }

            // Create symlink to global cache
            if version_mismatch {
                println!(
                    "  {} {} -> {} (using {})",
                    "↗".cyan(),
                    package,
                    "global cache".dimmed(),
                    cached_name.yellow()
                );
            } else {
                println!(
                    "  {} {} -> {}",
                    "↗".cyan(),
                    package,
                    "global cache".dimmed()
                );
            }
            symlink(cached, &local_link).context(format!("Failed to symlink {}", package))?;
        } else {
            // Package not found locally
            missing_packages.push(package.clone());
        }
    }

    // If there are missing packages, ask user if they want to install
    if !missing_packages.is_empty() {
        println!(
            "\n{} Missing {} package(s):",
            cli_output::ICON_WARN.yellow(),
            missing_packages.len()
        );
        for pkg in &missing_packages {
            println!("  • {}", pkg.yellow());
        }

        print!(
            "\n{} Install missing packages from registry? [Y/n]: ",
            "?".cyan()
        );
        io::stdout().flush()?;

        let mut input = String::new();
        io::stdin().read_line(&mut input)?;
        let input = input.trim().to_lowercase();

        if input.is_empty() || input == "y" || input == "yes" {
            // User wants to install
            println!("\n{} Installing packages...", cli_output::ICON_INFO.cyan());

            // Import registry client
            use crate::registry::RegistryClient;
            use crate::workspace;
            let client = RegistryClient::new();
            let target = workspace::detect_or_select_workspace(true)?;

            // Try to use structured dependencies from horus.yaml
            let horus_yaml_path = Path::new(HORUS_YAML);
            let use_structured_deps = horus_yaml_path.exists();

            // Get base directory for resolving relative paths (directory containing horus.yaml)
            let base_dir = horus_yaml_path.parent().or_else(|| Some(Path::new(".")));

            if use_structured_deps {
                // Parse with v2 to get DependencySpecs with source information
                match parse_horus_yaml_dependencies_v2(HORUS_YAML) {
                    Ok(dep_specs) => {
                        // Create a map of package name -> DependencySpec
                        let mut spec_map: std::collections::HashMap<
                            String,
                            crate::dependency_resolver::DependencySpec,
                        > = dep_specs
                            .into_iter()
                            .map(|spec| (spec.name.clone(), spec))
                            .collect();

                        for package in &missing_packages {
                            if let Some(spec) = spec_map.remove(package) {
                                print!("  {} Installing {}... ", cli_output::ICON_INFO.cyan(), package.yellow());
                                io::stdout().flush()?;

                                match client.install_dependency_spec(
                                    &spec,
                                    target.clone(),
                                    base_dir,
                                ) {
                                    Ok(_) => {
                                        println!("{}", cli_output::ICON_SUCCESS.green());
                                    }
                                    Err(e) => {
                                        println!("{}", cli_output::ICON_ERROR.red());
                                        eprintln!(
                                            "    {} Failed to install {}: {}",
                                            cli_output::ICON_ERROR.red(),
                                            package,
                                            e
                                        );
                                        bail!("Failed to install required dependency: {}", package);
                                    }
                                }
                            } else {
                                // Fallback to registry install if spec not found
                                print!(
                                    "  {} Installing {} (from registry)... ",
                                    cli_output::ICON_INFO.cyan(),
                                    package.yellow()
                                );
                                io::stdout().flush()?;
                                match client.install(package, None) {
                                    Ok(_) => println!("{}", cli_output::ICON_SUCCESS.green()),
                                    Err(e) => {
                                        println!("{}", cli_output::ICON_ERROR.red());
                                        bail!("Failed to install {}: {}", package, e);
                                    }
                                }
                            }
                        }
                    }
                    Err(_) => {
                        // Fallback to old parser
                        for package in &missing_packages {
                            print!("  {} Installing {}... ", cli_output::ICON_INFO.cyan(), package.yellow());
                            io::stdout().flush()?;

                            match client.install(package, None) {
                                Ok(_) => {
                                    println!("{}", cli_output::ICON_SUCCESS.green());
                                }
                                Err(e) => {
                                    println!("{}", cli_output::ICON_ERROR.red());
                                    eprintln!(
                                        "    {} Failed to install {}: {}",
                                        cli_output::ICON_ERROR.red(),
                                        package,
                                        e
                                    );
                                    bail!("Failed to install required dependency: {}", package);
                                }
                            }
                        }
                    }
                }
            } else {
                // No horus.yaml, use old behavior
                for package in &missing_packages {
                    print!("  {} Installing {}... ", cli_output::ICON_INFO.cyan(), package.yellow());
                    io::stdout().flush()?;

                    match client.install(package, None) {
                        Ok(_) => {
                            println!("{}", cli_output::ICON_SUCCESS.green());
                        }
                        Err(e) => {
                            println!("{}", cli_output::ICON_ERROR.red());
                            eprintln!("    {} Failed to install {}: {}", cli_output::ICON_ERROR.red(), package, e);
                            bail!("Failed to install required dependency: {}", package);
                        }
                    }
                }
            }

            println!("\n{} All dependencies installed successfully!", cli_output::ICON_SUCCESS.green());
        } else {
            // User declined
            println!(
                "\n{} Installation cancelled. Cannot proceed without dependencies.",
                cli_output::ICON_ERROR.red()
            );
            bail!(
                "Missing required dependencies: {}",
                missing_packages.join(", ")
            );
        }
    }

    Ok(())
}

pub(crate) fn find_cached_versions(cache_dir: &Path, package: &str) -> Result<Vec<PathBuf>> {
    let mut versions = Vec::new();

    if !cache_dir.exists() {
        return Ok(versions);
    }

    // Parse package name and version if specified (e.g., "horus_py@0.1.0" -> ("horus_py", Some("0.1.5")))
    let (base_package, requested_version) = if let Some(at_pos) = package.find('@') {
        (&package[..at_pos], Some(&package[at_pos + 1..]))
    } else {
        (package, None)
    };

    for entry in fs::read_dir(cache_dir)? {
        let entry = entry?;
        let name = entry.file_name();
        let name_str = name.to_string_lossy();

        // Match base package name
        if name_str == base_package || name_str.starts_with(&format!("{}@", base_package)) {
            // If a specific version was requested, prefer exact match
            if let Some(req_ver) = requested_version {
                if name_str == format!("{}@{}", base_package, req_ver) {
                    // Exact version match - prioritize this
                    versions.insert(0, entry.path());
                } else {
                    // Different version - add to list as fallback
                    versions.push(entry.path());
                }
            } else {
                // No specific version requested - add all
                versions.push(entry.path());
            }
        }
    }

    // Sort by version (newest first), but keep exact match at front if it exists
    if requested_version.is_some() && !versions.is_empty() {
        // First entry is exact match (if found), don't sort it out
        let exact_match = versions.first().cloned();
        let is_exact = exact_match.as_ref().is_some_and(|p| {
            p.file_name().and_then(|n| n.to_str()).is_some_and(|n| {
                requested_version.is_some_and(|v| n == format!("{}@{}", base_package, v))
            })
        });

        if is_exact {
            // Keep exact match at front, sort the rest
            let mut rest = versions.split_off(1);
            rest.sort_by(|a, b| b.cmp(a));
            versions.extend(rest);
        } else {
            // No exact match, sort all by version (newest first)
            versions.sort_by(|a, b| b.cmp(a));
        }
    } else {
        // Sort by version (newest first)
        versions.sort_by(|a, b| b.cmp(a));
    }

    Ok(versions)
}

pub(crate) fn home_dir() -> PathBuf {
    // Cross-platform home directory detection
    dirs::home_dir().unwrap_or_else(|| {
        // Fallback to temp directory if home not found
        std::env::temp_dir()
    })
}

#[derive(Debug, Clone, PartialEq)]
pub(crate) enum SystemPackageChoiceRun {
    UseSystem,
    InstallHORUS,
    Cancel,
}

pub(crate) fn prompt_system_cargo_choice_run(
    package_name: &str,
    system_version: &str,
) -> Result<SystemPackageChoiceRun> {
    use std::io::{self, Write};

    println!(
        "\n{} crates.io {} found in system (version: {})",
        "[WARNING]".yellow(),
        package_name.green(),
        system_version.cyan()
    );
    println!("\nWhat would you like to do?");
    println!("  [1] {} Use system binary (create reference)", cli_output::ICON_SUCCESS.green());
    println!(
        "  [2] {} Install to HORUS (isolated environment)",
        "".blue()
    );
    println!("  [3] {} Skip this package", "⊘".yellow());

    print!("\nChoice [1-3]: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    match input.trim() {
        "1" => Ok(SystemPackageChoiceRun::UseSystem),
        "2" => Ok(SystemPackageChoiceRun::InstallHORUS),
        "3" => Ok(SystemPackageChoiceRun::Cancel),
        _ => {
            println!("Invalid choice, defaulting to Install to HORUS");
            Ok(SystemPackageChoiceRun::InstallHORUS)
        }
    }
}

pub(crate) fn create_system_reference_cargo_run(
    package_name: &str,
    system_version: &str,
) -> Result<()> {
    println!("  {} Creating reference to system binary...", cli_output::ICON_SUCCESS.green());

    // Find actual system binary location
    let home = dirs::home_dir().ok_or_else(|| anyhow!("could not find home directory"))?;
    let cargo_bin = home.join(".cargo/bin").join(package_name);

    if !cargo_bin.exists() {
        bail!("System binary not found at expected location");
    }

    let packages_dir = PathBuf::from(".horus/packages");
    fs::create_dir_all(&packages_dir)?;

    let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
    let metadata = serde_json::json!({
        "name": package_name,
        "version": system_version,
        "source": "System",
        "system_path": cargo_bin.display().to_string(),
        "package_type": "CratesIO"
    });

    fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

    // Create symlink in .horus/bin to system binary (Unix) or copy on Windows
    let bin_dir = PathBuf::from(".horus/bin");
    fs::create_dir_all(&bin_dir)?;

    let bin_link = bin_dir.join(package_name);
    if bin_link.exists() {
        fs::remove_file(&bin_link)?;
    }

    #[cfg(unix)]
    {
        symlink(&cargo_bin, &bin_link)?;
    }
    #[cfg(windows)]
    {
        // On Windows, create a .cmd wrapper instead of symlink
        let cmd_link = bin_dir.join(format!("{}.cmd", package_name));
        fs::write(&cmd_link, format!("@\"{}\"\r\n", cargo_bin.display()))?;
    }

    println!(
        "  {} Using system binary at {}",
        cli_output::ICON_SUCCESS.green(),
        cargo_bin.display()
    );
    println!(
        "  {} Reference created: {}",
        cli_output::ICON_INFO.cyan(),
        metadata_file.display()
    );
    println!("  {} Binary linked: {}", cli_output::ICON_INFO.cyan(), bin_link.display());

    Ok(())
}

pub(crate) fn detect_system_python_package(package_name: &str) -> Result<Option<String>> {
    use std::process::Command;

    // Check if package is installed in system Python using pip show
    let output = Command::new("python3")
        .args(["-m", "pip", "show", package_name])
        .output();

    if let Ok(output) = output {
        if output.status.success() {
            let stdout = String::from_utf8_lossy(&output.stdout);
            // Parse version from pip show output
            for line in stdout.lines() {
                if line.starts_with("Version:") {
                    let version = line.trim_start_matches("Version:").trim().to_string();
                    return Ok(Some(version));
                }
            }
            // Package found but version unknown
            return Ok(Some("unknown".to_string()));
        }
    }

    Ok(None)
}

pub(crate) fn prompt_system_package_choice_run(
    package_name: &str,
    system_version: &str,
) -> Result<SystemPackageChoiceRun> {
    use std::io::{self, Write};

    println!(
        "\n{} PyPI package {} found in system (version: {})",
        "[WARNING]".yellow(),
        package_name.green(),
        system_version.cyan()
    );
    println!("\nWhat would you like to do?");
    println!("  [1] {} Use system package (create reference)", cli_output::ICON_SUCCESS.green());
    println!(
        "  [2] {} Install to HORUS (isolated environment)",
        "".blue()
    );
    println!("  [3] {} Skip this package", "⊘".yellow());

    print!("\nChoice [1-3]: ");
    io::stdout().flush()?;

    let mut input = String::new();
    io::stdin().read_line(&mut input)?;

    match input.trim() {
        "1" => Ok(SystemPackageChoiceRun::UseSystem),
        "2" => Ok(SystemPackageChoiceRun::InstallHORUS),
        "3" => Ok(SystemPackageChoiceRun::Cancel),
        _ => {
            println!("Invalid choice, defaulting to Install to HORUS");
            Ok(SystemPackageChoiceRun::InstallHORUS)
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use tempfile::TempDir;

    // ─── find_cached_versions Tests ───

    #[test]
    fn find_cached_versions_empty_cache() {
        let cache = TempDir::new().unwrap();
        let result = find_cached_versions(cache.path(), "my_package").unwrap();
        assert!(result.is_empty(), "Empty cache should return no versions");
    }

    #[test]
    fn find_cached_versions_nonexistent_dir() {
        let result =
            find_cached_versions(Path::new("/tmp/nonexistent_horus_cache_xyz"), "pkg").unwrap();
        assert!(
            result.is_empty(),
            "Non-existent cache dir should return empty"
        );
    }

    #[test]
    fn find_cached_versions_finds_unversioned() {
        let cache = TempDir::new().unwrap();
        // Create a package directory (just the name, no version)
        fs::create_dir_all(cache.path().join("my_package")).unwrap();

        let result = find_cached_versions(cache.path(), "my_package").unwrap();
        assert_eq!(result.len(), 1);
        assert!(result[0].ends_with("my_package"));
    }

    #[test]
    fn find_cached_versions_finds_versioned() {
        let cache = TempDir::new().unwrap();
        fs::create_dir_all(cache.path().join("my_package@1.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("my_package@2.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("my_package@1.1.0")).unwrap();

        let result = find_cached_versions(cache.path(), "my_package").unwrap();
        assert_eq!(result.len(), 3, "Should find all 3 versioned directories");
    }

    #[test]
    fn find_cached_versions_sorted_newest_first() {
        let cache = TempDir::new().unwrap();
        fs::create_dir_all(cache.path().join("pkg@1.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("pkg@2.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("pkg@1.5.0")).unwrap();

        let result = find_cached_versions(cache.path(), "pkg").unwrap();
        assert_eq!(result.len(), 3);
        // Sorted by path descending — 2.0.0 > 1.5.0 > 1.0.0
        let names: Vec<String> = result
            .iter()
            .map(|p| p.file_name().unwrap().to_string_lossy().to_string())
            .collect();
        assert_eq!(names[0], "pkg@2.0.0");
        assert_eq!(names[1], "pkg@1.5.0");
        assert_eq!(names[2], "pkg@1.0.0");
    }

    #[test]
    fn find_cached_versions_exact_match_prioritized() {
        let cache = TempDir::new().unwrap();
        fs::create_dir_all(cache.path().join("pkg@1.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("pkg@2.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("pkg@3.0.0")).unwrap();

        // Request exact version 1.0.0 — should be first despite being the oldest
        let result = find_cached_versions(cache.path(), "pkg@1.0.0").unwrap();
        assert!(!result.is_empty());
        let first_name = result[0].file_name().unwrap().to_string_lossy().to_string();
        assert_eq!(first_name, "pkg@1.0.0", "Exact match should be first");
    }

    #[test]
    fn find_cached_versions_no_match_for_missing_version() {
        let cache = TempDir::new().unwrap();
        fs::create_dir_all(cache.path().join("pkg@1.0.0")).unwrap();

        // Request version 3.0.0 which doesn't exist
        let result = find_cached_versions(cache.path(), "pkg@3.0.0").unwrap();
        // Still returns pkg@1.0.0 as a fallback (not empty — it found the base package)
        // But exact match won't be first
        for path in &result {
            let name = path.file_name().unwrap().to_string_lossy().to_string();
            assert_ne!(
                name, "pkg@3.0.0",
                "Should not fabricate non-existent version"
            );
        }
    }

    #[test]
    fn find_cached_versions_ignores_unrelated_packages() {
        let cache = TempDir::new().unwrap();
        fs::create_dir_all(cache.path().join("my_pkg@1.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("other_pkg@2.0.0")).unwrap();
        fs::create_dir_all(cache.path().join("my_pkg_extra@1.0.0")).unwrap();

        let result = find_cached_versions(cache.path(), "my_pkg").unwrap();
        // Should find my_pkg@1.0.0 but NOT other_pkg@2.0.0
        // my_pkg_extra@1.0.0 should also be excluded (different base name)
        for path in &result {
            let name = path.file_name().unwrap().to_string_lossy().to_string();
            assert!(
                name == "my_pkg" || name.starts_with("my_pkg@"),
                "Should only find my_pkg variants, got: {}",
                name
            );
        }
    }

    // ─── home_dir Tests ───

    #[test]
    fn home_dir_returns_valid_path() {
        let home = home_dir();
        assert!(
            home.exists(),
            "home_dir should return an existing directory"
        );
        assert!(home.is_dir(), "home_dir should return a directory");
    }

    // ─── SystemPackageChoiceRun Tests ───

    #[test]
    fn system_package_choice_variants() {
        let use_system = SystemPackageChoiceRun::UseSystem;
        let install_horus = SystemPackageChoiceRun::InstallHORUS;
        let cancel = SystemPackageChoiceRun::Cancel;

        assert_eq!(use_system, SystemPackageChoiceRun::UseSystem);
        assert_eq!(install_horus, SystemPackageChoiceRun::InstallHORUS);
        assert_eq!(cancel, SystemPackageChoiceRun::Cancel);
        assert_ne!(use_system, cancel);
        assert_ne!(install_horus, cancel);
    }
}

pub(crate) fn create_system_reference_python_run(
    package_name: &str,
    system_version: &str,
) -> Result<()> {
    println!("  {} Creating reference to system package...", cli_output::ICON_SUCCESS.green());

    let packages_dir = PathBuf::from(".horus/packages");
    fs::create_dir_all(&packages_dir)?;

    let metadata_file = packages_dir.join(format!("{}.system.json", package_name));
    let metadata = serde_json::json!({
        "name": package_name,
        "version": system_version,
        "source": "System",
        "package_type": "PyPI"
    });

    fs::write(&metadata_file, serde_json::to_string_pretty(&metadata)?)?;

    println!("  {} Using system package", cli_output::ICON_SUCCESS.green());
    println!(
        "  {} Reference created: {}",
        cli_output::ICON_INFO.cyan(),
        metadata_file.display()
    );

    Ok(())
}
