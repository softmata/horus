use super::deps::{split_dependencies_with_context, CargoPackage, PipPackage};
use crate::cargo_utils::detect_system_cargo_binary;
use crate::cli_output;
use crate::lockfile::{hash_config, HorusLockfile, HORUS_LOCK};
use crate::manifest::HORUS_TOML;
use crate::version;
use anyhow::{anyhow, bail, Context, Result};
use colored::*;
use std::collections::HashSet;
use std::fs;
use std::io::{self, Write};
/// Cross-platform symlink creation (delegates to horus_sys::fs).
fn symlink(src: &Path, dst: &Path) -> Result<()> {
    horus_sys::fs::symlink(src, dst)
}
use std::path::{Path, PathBuf};
use std::process::Command;

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

    // What the interpreter has to be able to import once this function has
    // finished. Every branch that leaves a link behind records here, including
    // the "(already linked)" fast paths: a link made by a previous run is
    // exactly as unimportable as one made by this one if the path is wrong,
    // and it is the second run onwards that users spend their time in.
    let mut linked: Vec<(String, Vec<String>)> = Vec::new();

    for pkg in &packages {
        // Check if package exists in system first
        if let Ok(Some(system_version)) = detect_system_python_package(&pkg.name) {
            // Auto-use system package for horus-robotics (core dependency)
            if pkg.name == "horus-robotics" {
                create_system_reference_python_run(&pkg.name, &system_version)?;
                continue;
            }

            let local_link = local_packages.join(&pkg.name);

            // Skip only if what is already there still works. `read_link()`
            // answers Ok for a dangling symlink for as long as the link file
            // exists, so this gate used to call a link into a deleted cache
            // "already handled" and move on.
            if python_link_is_usable(&local_link) {
                linked.push((pkg.name.clone(), top_level_modules(&local_link)));
                continue;
            }
            clear_broken_link(&local_link);

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

        // A link is a cache hit only if it still resolves to a usable package.
        //
        // `local_link.exists()` follows the symlink and so answers false once
        // the cache directory is gone — but `read_link()` answers Ok for as
        // long as the link file itself is there, so the old `||` accepted a
        // dangling link forever. HORUS printed the green tick and
        // "(already linked)" over a link to nothing, wrote horus.lock, and the
        // node then died at `import`. Deleting a cache directory is the remedy
        // this very function prints a few lines below, so this is a state users
        // are actively told to create.
        if python_link_is_usable(&local_link) {
            println!(
                "  {} {} (already linked)",
                cli_output::ICON_SUCCESS.green(),
                pkg.name
            );
            linked.push((pkg.name.clone(), top_level_modules(&local_link)));
            continue;
        }
        if clear_broken_link(&local_link) {
            println!(
                "  {} {} was linked to a package that is no longer there — reinstalling",
                cli_output::ICON_WARN.yellow(),
                pkg.name
            );
        }

        // If not usably cached, install to the global cache.
        //
        // `exists()` is not the question. `create_dir_all` below runs *before*
        // pip, so a pip failure — or a kill, or a full disk — leaves the
        // directory behind with nothing in it. On every later run that empty
        // directory answered `exists()` with true, so HORUS reported
        //
        //     ↗ horus-robotics -> global cache
        //     * Linked horus-robotics
        //
        // and linked an empty directory. The node then died with "No module
        // named 'horus'", pointing at the import rather than at the install
        // that never happened. Poisoned once, poisoned until someone deletes
        // ~/.horus/cache by hand.
        if !cache_is_usable(&pkg_cache_dir) {
            println!(
                "  {} Installing {} to global cache...",
                cli_output::ICON_INFO.cyan(),
                pkg.name
            );

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
                // Take the empty directory with us. Leaving it is what turned a
                // failed install into a permanent silent one.
                let _ = fs::remove_dir_all(&pkg_cache_dir);
                crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::pip_error_hint(
                    &stderr,
                ));
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

        // Say what was linked, not just that something was. "Linked" printed
        // over an empty directory is the message that made this defect invisible.
        let modules = top_level_modules(&pkg_cache_dir);
        if modules.is_empty() {
            println!(
                "  {} Linked {} — but it provides no importable module. \
                 Delete {} and re-run to reinstall.",
                cli_output::ICON_WARN.yellow(),
                pkg.name,
                pkg_cache_dir.display()
            );
        } else {
            println!(
                "  {} Linked {} ({})",
                cli_output::ICON_SUCCESS.green(),
                pkg.name,
                modules.join(", ")
            );
            linked.push((pkg.name.clone(), modules));
        }
    }

    verify_linked_packages_import(&python_cmd, &linked)?;

    Ok(())
}

/// Ask the interpreter that will run the node whether it can import what was
/// just linked, and fail here if it cannot.
///
/// Reporting success four times over — `↗ global cache`, `Linked`,
/// `Updated horus.lock`, `Dependencies locked` — was the whole shape of this
/// defect: every one of those lines described a step this code had taken, and
/// not one of them asked Python whether the result was usable. It was not.
/// PYTHONPATH was assembled from a different cache root than the one pip had
/// written to, so `horus run` printed four green lines and then died on
/// `import horus`, pointing the user at their own source file.
///
/// This is the one check a path bug cannot walk past, because it runs the real
/// import through the real PYTHONPATH. It is deliberately placed at the link
/// step: an install that cannot be imported has not succeeded, and saying so
/// here names the package and the interpreter instead of leaving a
/// `ModuleNotFoundError` to be read as the user's mistake.
///
/// All packages are probed in one child interpreter so that verifying on every
/// run — including the runs that only re-use existing links — costs one process
/// start rather than one per dependency.
fn verify_linked_packages_import(python_cmd: &str, linked: &[(String, Vec<String>)]) -> Result<()> {
    let modules: Vec<String> = linked
        .iter()
        .flat_map(|(_, modules)| modules.iter().cloned())
        .collect();
    if modules.is_empty() {
        // Nothing importable was linked. A distribution of pure scripts has
        // nothing to check, and the empty-cache case is already reported above.
        return Ok(());
    }

    let python_path = super::run_python::build_python_path()?;
    let (interpreter, failures) = probe_imports(python_cmd, &python_path, &modules)?;
    if failures.is_empty() {
        return Ok(());
    }

    for (package, modules) in linked {
        let failed: Vec<&(String, String)> = failures
            .iter()
            .filter(|(module, _)| modules.contains(module))
            .collect();
        if failed.is_empty() {
            continue;
        }

        if failed.len() < modules.len() {
            // At least one of this distribution's modules imported, so the path
            // is wired up. Distributions ship top-level names that need extras
            // HORUS was never asked for; that is the package's business, not a
            // broken install.
            for (module, reason) in failed {
                println!(
                    "  {} {} did not import: {}",
                    cli_output::ICON_WARN.yellow(),
                    module,
                    reason
                );
            }
            continue;
        }

        let (module, reason) = failed[0];
        let install = format!("{} -m pip install {}", python_cmd, package);
        // H041 (`import-error`) is the catalogued code for a Python import that
        // failed; what is specific here is the fix, which names the distribution
        // and the interpreter rather than the module the node happened to write.
        crate::error_wrapper::emit_diagnostics(&[crate::error_wrapper::Diagnostic::new(
            "python",
            "H041",
            "Linked package is not importable",
            format!(
                "Linked distribution {}, but `import {}` failed in {}: {}\n\
                 Install it into that interpreter with:\n  {}",
                package,
                module,
                interpreter,
                reason,
                install.green()
            ),
        )
        .with_fix(crate::error_wrapper::Fix::Command {
            command: install.clone(),
        })]);
        bail!(
            "Linked distribution {}, but `import {}` failed in {}. Install it with `{}`.",
            package,
            module,
            interpreter,
            install
        );
    }

    Ok(())
}

/// Try to import each module in a child interpreter.
///
/// Returns the interpreter's own `sys.executable` — so the error can name the
/// Python that actually failed rather than the `python3` we spelled on the
/// command line — together with the modules that did not import and why.
///
/// The module names travel in `argv`, never interpolated into the source
/// string, for the same reason `create_python_wrapper` passes the node path
/// through the environment: a name that reached this from a manifest must not
/// be able to become code.
fn probe_imports(
    python_cmd: &str,
    python_path: &str,
    modules: &[String],
) -> Result<(String, Vec<(String, String)>)> {
    const PROBE: &str = r#"import importlib, sys
print(sys.executable)
for name in sys.argv[1:]:
    try:
        importlib.import_module(name)
    except BaseException as exc:
        print("%s	%s: %s" % (name, type(exc).__name__, exc))
"#;

    let output = Command::new(python_cmd)
        .arg("-c")
        .arg(PROBE)
        .args(modules)
        .env("PYTHONPATH", python_path)
        .output()
        .context("Failed to run the post-install import check")?;

    let stdout = String::from_utf8_lossy(&output.stdout);
    let mut lines = stdout.lines();
    let interpreter = lines.next().unwrap_or(python_cmd).trim().to_string();

    let mut failures = Vec::new();
    for line in lines {
        let (module, reason) = line.split_once('\t').unwrap_or((line, "import failed"));
        failures.push((module.to_string(), reason.to_string()));
    }

    // A probe that could not run at all tells us nothing about the package.
    // Reporting every module as failed on, say, a killed interpreter would
    // turn a working install into a hard error, so treat it as no evidence.
    if !output.status.success() && failures.is_empty() {
        return Ok((interpreter, Vec::new()));
    }

    Ok((interpreter, failures))
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

            // Same gate as the pypi path: a dangling link is not a hit.
            if cargo_link_is_usable(&local_link) {
                continue;
            }
            clear_broken_link(&local_link);

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

        // A link is a hit only if the binary it points at is still there;
        // `read_link()` says Ok about a link into a deleted cache too.
        if cargo_link_is_usable(&local_link) {
            println!(
                "  {} {} (already linked)",
                cli_output::ICON_SUCCESS.green(),
                pkg.name
            );
            continue;
        }
        if clear_broken_link(&local_link) {
            println!(
                "  {} {} was linked to a binary that is no longer there — reinstalling",
                cli_output::ICON_WARN.yellow(),
                pkg.name
            );
        }

        // If not usably cached, install to the global cache.
        //
        // Same reasoning as the pypi path below: `create_dir_all` runs before
        // `cargo install`, so a failure leaves an empty directory that
        // `exists()` reads as a cache hit on every later run.
        if !cargo_cache_is_usable(&pkg_cache_dir) {
            println!(
                "  {} Installing {} to global cache...",
                cli_output::ICON_INFO.cyan(),
                pkg.name
            );

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
                // Do not leave the empty directory: it would be read as a hit.
                let _ = fs::remove_dir_all(&pkg_cache_dir);
                crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::cargo_error_hint(
                    &stderr,
                ));
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

    // ── Lockfile: check if resolution can be skipped ──
    let lock_path = Path::new(HORUS_LOCK);
    let toml_path = Path::new(HORUS_TOML);
    let config_hash = {
        let toml_content = if toml_path.exists() {
            fs::read_to_string(toml_path).ok().unwrap_or_default()
        } else {
            String::new()
        };
        // Include detected dependencies in the hash so that new imports
        // (e.g. `import numpy`) invalidate the lockfile even when horus.toml
        // hasn't changed.
        let mut sorted_deps: Vec<&str> = dependencies.iter().map(String::as_str).collect();
        sorted_deps.sort_unstable();
        let hash_input = format!("{}\n{}", toml_content, sorted_deps.join("\n"));
        Some(hash_config(&hash_input))
    };

    // Split dependencies into HORUS packages, pip packages, and cargo packages.
    // This happens before the lockfile check because the lockfile's answer is
    // only trustworthy if the packages it stands for are still installed, and
    // knowing which those are means splitting first. The split is pure and
    // local — no network, no filesystem.
    let (horus_packages, pip_packages, cargo_packages) =
        split_dependencies_with_context(dependencies.clone(), context_language);

    if lock_path.exists() {
        if let Ok(existing_lock) = HorusLockfile::load_from(lock_path) {
            if let Some(ref hash) = config_hash {
                if !existing_lock.is_stale(hash) {
                    // A matching config hash says the *inputs* have not
                    // changed. horus.lock pins no packages and records no
                    // paths, so it says nothing whatever about whether the
                    // outputs are still on disk. Returning here on the hash
                    // alone meant that deleting a cache directory — the remedy
                    // install_pip_packages itself prints — produced
                    //
                    //     * Dependencies locked (horus.lock is up-to-date)
                    //     Node execution failed: No module named 'cowsay'
                    //
                    // with the code that would have reinstalled it never
                    // reached. The lock may skip work; it may not skip
                    // reality.
                    let missing = unmaterialized_dependencies(
                        Path::new("."),
                        &horus_packages,
                        &pip_packages,
                        &cargo_packages,
                        context_language,
                    );
                    if missing.is_empty() {
                        log::info!("Lockfile is up-to-date, using pinned versions");
                        eprintln!(
                            "  {} Dependencies locked (horus.lock is up-to-date)",
                            cli_output::ICON_SUCCESS.green()
                        );
                        return Ok(());
                    }
                    log::info!(
                        "Lockfile is up-to-date but {} dependency/ies are not installed",
                        missing.len()
                    );
                    eprintln!(
                        "  {} horus.lock is up-to-date, but {} not installed — re-resolving",
                        cli_output::ICON_WARN.yellow(),
                        if missing.len() == 1 {
                            format!("{} is", missing[0])
                        } else {
                            format!("{} are", missing.join(", "))
                        }
                    );
                } else {
                    log::info!("Lockfile is stale, re-resolving dependencies");
                }
            }
        }
    }

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

    // ── Lockfile: write after successful resolution ──
    write_lockfile(&config_hash)?;

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

        // Skip only if the link still resolves to something. The pypi and
        // crates.io paths had the same gate and the same defect: a link whose
        // target has been deleted was reported as a success.
        if horus_link_is_usable(&local_link) {
            println!(
                "  {} {} (already linked)",
                cli_output::ICON_SUCCESS.green(),
                package
            );
            continue;
        }
        if clear_broken_link(&local_link) {
            println!(
                "  {} {} was linked to a package that is no longer there — re-resolving",
                cli_output::ICON_WARN.yellow(),
                package
            );
        }

        // Check global cache. A cache entry that exists but holds nothing is
        // not a hit: `find_cached_versions` matches on the directory *name*,
        // so a failed or interrupted install answers it just as well as a
        // finished one.
        let cached_versions: Vec<PathBuf> = find_cached_versions(&global_cache, package)?
            .into_iter()
            .filter(|dir| horus_cache_is_usable(dir))
            .collect();

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

                    // Check if the symlink already resolves (a dangling one
                    // does not count — see above).
                    if horus_link_is_usable(&horus_link) {
                        println!(
                            "  {} {} (already linked)",
                            cli_output::ICON_SUCCESS.green(),
                            package
                        );
                        continue;
                    }
                    clear_broken_link(&horus_link);

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
            // Say what arrived, not just that something did — the pypi path
            // reports its modules for the same reason.
            println!(
                "  {} Linked {} ({})",
                cli_output::ICON_SUCCESS.green(),
                package,
                cached_name
            );
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
            println!("  - {}", pkg.yellow());
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
            let client = RegistryClient::new();

            // Install missing packages from the HORUS registry.
            for package in &missing_packages {
                print!(
                    "  {} Installing {}... ",
                    cli_output::ICON_INFO.cyan(),
                    package.yellow()
                );
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

            println!(
                "\n{} All dependencies installed successfully!",
                cli_output::ICON_SUCCESS.green()
            );
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
    println!(
        "  [1] {} Use system binary (create reference)",
        cli_output::ICON_SUCCESS.green()
    );
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
    println!(
        "  {} Creating reference to system binary...",
        cli_output::ICON_SUCCESS.green()
    );

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
    println!(
        "  {} Binary linked: {}",
        cli_output::ICON_INFO.cyan(),
        bin_link.display()
    );

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
    println!(
        "  [1] {} Use system package (create reference)",
        cli_output::ICON_SUCCESS.green()
    );
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

/// Write a lockfile recording the current config hash and the packages that
/// resolution actually put on disk.
///
/// Dependencies are also tracked by native lockfiles (Cargo.lock, etc.), but
/// the horus lockfile used to record *only* whether the config had changed:
/// `[[package]]` was always absent, so a file that answered "up to date" had
/// never named a single version, and a second machine reading it learned
/// nothing about what to install. A config hash is a staleness check, not a
/// reproducibility record.
fn write_lockfile(config_hash: &Option<String>) -> Result<()> {
    let lock_path = Path::new(HORUS_LOCK);

    // Load-then-mutate. `HorusLockfile::new()` produced a BLANK lockfile, so
    // every `horus run`/`horus build` whose config hash had changed silently
    // replaced the file with one containing only that hash — discarding the
    // toolchain pins, system-dep pins, features and package set that
    // `horus lock` and `horus doctor` had recorded. The reproducibility
    // guarantee the lockfile exists to provide was erased by the ordinary
    // build path.
    let mut lockfile = HorusLockfile::load_from(lock_path).unwrap_or_default();
    lockfile.config_hash = config_hash.clone();

    for (name, version, source) in resolved_package_pins(Path::new(".")) {
        lockfile.pin(&name, &version, &source, None);
    }

    lockfile
        .save_to(lock_path)
        .context("Failed to write horus.lock")?;

    log::info!("Wrote lockfile: {}", lock_path.display());
    eprintln!(
        "  {} Updated {}",
        cli_output::ICON_SUCCESS.green(),
        HORUS_LOCK
    );

    Ok(())
}

/// The packages `.horus/` says are installed, as `(name, version, source)`.
///
/// Read back off disk rather than collected as the installers run, so the
/// lockfile records what is linked into this project — including packages a
/// previous run resolved and this one skipped as already-linked — rather than
/// what this particular invocation happened to do. That is the set a second
/// machine has to reproduce.
///
/// Anything whose version cannot be established is left out: a pin that says
/// `version = "latest"` is not a pin, and writing one would make the lockfile
/// look authoritative while promising nothing.
fn resolved_package_pins(project_dir: &Path) -> Vec<(String, String, String)> {
    let mut pins = Vec::new();

    if let Ok(entries) = fs::read_dir(project_dir.join(".horus/packages")) {
        for entry in entries.flatten() {
            let path = entry.path();
            let file_name = entry.file_name().to_string_lossy().to_string();

            // A reference to a package already installed in the system
            // interpreter — no cache directory, the version is in the marker.
            if let Some(name) = file_name.strip_suffix(".system.json") {
                if let Some(version) = system_reference_version(&path) {
                    pins.push((name.to_string(), version, "pypi".to_string()));
                }
                continue;
            }

            // Everything else is a link into the global cache, whose directory
            // name carries both the distribution and the version.
            let Some(target) = link_target_name(&path) else {
                continue;
            };
            if let Some((name, version)) = target.strip_prefix("pypi_").and_then(split_name_version)
            {
                // pip is asked for `latest` when the manifest names no version,
                // so the directory name cannot answer this one — the installed
                // dist-info can.
                let version = if version == "latest" {
                    match dist_info_version(&path) {
                        Some(v) => v,
                        None => continue,
                    }
                } else {
                    version.to_string()
                };
                pins.push((name.to_string(), version, "pypi".to_string()));
            } else if let Some((name, version)) = split_name_version(target.as_str()) {
                pins.push((
                    name.to_string(),
                    version.to_string(),
                    "registry".to_string(),
                ));
            }
        }
    }

    if let Ok(entries) = fs::read_dir(project_dir.join(".horus/bin")) {
        for entry in entries.flatten() {
            // `.horus/bin/<name>` links to `<cache>/cratesio_<name>@<ver>/bin/<name>`,
            // so the version lives two levels up from the link target.
            let Ok(target) = fs::read_link(entry.path()) else {
                continue;
            };
            let Some(pkg_dir) = target.parent().and_then(|bin| bin.parent()) else {
                continue;
            };
            let dir_name = pkg_dir.file_name().unwrap_or_default().to_string_lossy();
            if let Some((name, version)) = dir_name
                .strip_prefix("cratesio_")
                .and_then(split_name_version)
                .filter(|(_, version)| *version != "latest")
            {
                pins.push((
                    name.to_string(),
                    version.to_string(),
                    "crates.io".to_string(),
                ));
            }
        }
    }

    pins.sort();
    pins.dedup();
    pins
}

/// Split a cache directory's `name@version` suffix.
fn split_name_version(dir_name: &str) -> Option<(&str, &str)> {
    dir_name
        .rsplit_once('@')
        .filter(|(name, _)| !name.is_empty())
}

/// The cache directory a `.horus/packages` entry links to, by name.
fn link_target_name(link: &Path) -> Option<String> {
    let target = fs::read_link(link).ok()?;
    Some(target.file_name()?.to_string_lossy().to_string())
}

/// The version recorded in a `<name>.system.json` reference marker.
fn system_reference_version(marker: &Path) -> Option<String> {
    let content = fs::read_to_string(marker).ok()?;
    let value: serde_json::Value = serde_json::from_str(&content).ok()?;
    value
        .get("version")
        .and_then(|v| v.as_str())
        .filter(|v| !v.is_empty() && *v != "unknown")
        .map(|v| v.to_string())
}

/// The version pip actually installed, from the `*.dist-info` it leaves behind.
///
/// Wheel metadata directories are named `{distribution}-{version}.dist-info`,
/// which is the only place the resolved version is written down when the
/// manifest asked for no particular one.
fn dist_info_version(pkg_dir: &Path) -> Option<String> {
    let entries = fs::read_dir(pkg_dir).ok()?;
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if let Some(stem) = name.strip_suffix(".dist-info") {
            if let Some((_, version)) = stem.rsplit_once('-') {
                if !version.is_empty() {
                    return Some(version.to_string());
                }
            }
        }
    }
    None
}

pub(crate) fn create_system_reference_python_run(
    package_name: &str,
    system_version: &str,
) -> Result<()> {
    println!(
        "  {} Creating reference to system package...",
        cli_output::ICON_SUCCESS.green()
    );

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

    println!(
        "  {} Using system package",
        cli_output::ICON_SUCCESS.green()
    );
    println!(
        "  {} Reference created: {}",
        cli_output::ICON_INFO.cyan(),
        metadata_file.display()
    );

    Ok(())
}

#[cfg(test)]
mod tests {

    // ── Cache completeness (PATH-3) ────────────────────────────────────────
    //
    // `create_dir_all` runs before pip, so a failed install leaves the cache
    // directory behind empty. The gate used to be `!pkg_cache_dir.exists()`,
    // which read that empty directory as a hit and linked it — the node then
    // died with "No module named 'horus'", pointing at the import rather than
    // at the install that never happened, and stayed poisoned until someone
    // deleted ~/.horus/cache by hand.

    /// The helper is only worth having if the install path consults it.
    ///
    /// The first version of these tests exercised `cache_is_usable` directly
    /// and passed with the call site reverted to `!pkg_cache_dir.exists()` —
    /// verifying the answer while the question went unasked.
    #[test]
    fn the_install_path_gates_on_cache_usability_not_existence() {
        // Only the code, not this module: an assertion that searches for a
        // string it also contains matches itself and always fails.
        let full = include_str!("install.rs");
        let src = &full[..full.find("\nmod tests {").unwrap_or(full.len())];
        assert!(
            src.contains("if !cache_is_usable(&pkg_cache_dir)"),
            "the pypi install path must decide on whether the cache holds a \
             package, not on whether the directory exists — an empty directory \
             is exactly what a failed pip install leaves behind"
        );
        assert!(
            src.contains("if !cargo_cache_is_usable(&pkg_cache_dir)"),
            "the crates.io install path has the same defect and needs the same \
             gate — `cargo install --root` leaves an empty directory too"
        );
        assert!(
            !src.contains("if !pkg_cache_dir.exists() {"),
            "the existence check is the defect, in either install path"
        );
    }

    /// A failed install must not leave the directory behind, or the next run
    /// finds it and treats it as a hit.
    #[test]
    fn a_failed_install_removes_the_directory_it_created() {
        let full = include_str!("install.rs");
        let src = &full[..full.find("\nmod tests {").unwrap_or(full.len())];
        for marker in ["pip install failed for", "cargo install failed for"] {
            let at = src
                .find(marker)
                .unwrap_or_else(|| panic!("the failure path for {marker:?} must exist"));
            let before = &src[at.saturating_sub(400)..at];
            assert!(
                before.contains("remove_dir_all(&pkg_cache_dir)"),
                "leaving the empty directory is what turned one failed install \
                 into a permanent silent one ({marker})"
            );
        }
    }

    #[test]
    fn a_cargo_cache_without_binaries_is_not_a_hit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("cratesio_thing@latest");
        std::fs::create_dir_all(dir.join("bin")).expect("mkdir");
        std::fs::write(dir.join("metadata.json"), "{}").expect("write");
        assert!(
            !super::cargo_cache_is_usable(&dir),
            "an empty bin/ means cargo install produced nothing"
        );
        std::fs::write(dir.join("bin/thing"), "").expect("write");
        assert!(super::cargo_cache_is_usable(&dir));
    }

    #[test]
    fn an_empty_cache_directory_is_not_a_cache_hit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("pypi_thing@latest");
        std::fs::create_dir_all(&dir).expect("mkdir");
        assert!(
            !super::cache_is_usable(&dir),
            "an empty directory is what a failed pip install leaves behind"
        );
    }

    /// An interrupted run can leave the marker with nothing beside it.
    #[test]
    fn metadata_alone_is_not_a_package() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("pypi_thing@latest");
        std::fs::create_dir_all(&dir).expect("mkdir");
        std::fs::write(dir.join("metadata.json"), "{}").expect("write");
        assert!(!super::cache_is_usable(&dir));
    }

    /// pip's own bookkeeping is not an importable module either.
    #[test]
    fn dist_info_alone_is_not_a_package() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("pypi_thing@latest");
        std::fs::create_dir_all(dir.join("thing-1.0.dist-info")).expect("mkdir");
        std::fs::write(dir.join("metadata.json"), "{}").expect("write");
        assert!(
            !super::cache_is_usable(&dir),
            "a .dist-info directory records an install; it does not provide one"
        );
    }

    #[test]
    fn a_real_package_is_a_cache_hit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("pypi_horus-robotics@latest");
        std::fs::create_dir_all(dir.join("horus")).expect("mkdir");
        std::fs::write(dir.join("horus/__init__.py"), "").expect("write");
        std::fs::write(dir.join("metadata.json"), "{}").expect("write");
        assert!(super::cache_is_usable(&dir));
    }

    /// The reported name is what is importable, not the package name — they
    /// differ routinely, and `horus-robotics` installing `horus` is the case
    /// that matters here.
    #[test]
    fn the_importable_modules_are_reported_not_the_package_name() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("pypi_horus-robotics@latest");
        std::fs::create_dir_all(dir.join("horus")).expect("mkdir");
        std::fs::write(dir.join("horus/__init__.py"), "").expect("write");
        std::fs::create_dir_all(dir.join("horus_robotics-0.2.2.dist-info")).expect("mkdir");
        std::fs::write(dir.join("metadata.json"), "{}").expect("write");
        std::fs::write(dir.join("helper.py"), "").expect("write");

        let mut got = super::top_level_modules(&dir);
        got.sort();
        assert_eq!(got, vec!["helper".to_string(), "horus".to_string()]);
    }

    // ── Links and the lockfile fast path (PATH-3, second round) ──────────
    //
    // The cache gate above was fixed, and then two gates in front of it kept
    // the defect alive: `link.exists() || link.read_link().is_ok()` accepted a
    // dangling symlink as "(already linked)", and the lockfile returned
    // "Dependencies locked" on a config hash alone, before any of this code
    // was reached. Deleting a cache directory — the remedy this module itself
    // prints — reproduced the original failure verbatim: a green tick, then
    // "No module named 'cowsay'".

    /// A link whose target is gone is the trap: `exists()` is false (it
    /// follows the link) but `read_link()` is still Ok, so the old
    /// `exists() || read_link().is_ok()` gate was true forever.
    #[test]
    fn a_link_into_a_deleted_cache_is_not_a_hit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let cache = tmp.path().join("pypi_cowsay@latest");
        std::fs::create_dir_all(cache.join("cowsay")).expect("mkdir");
        std::fs::write(cache.join("cowsay/__init__.py"), "").expect("write");
        std::fs::write(cache.join("metadata.json"), "{}").expect("write");

        let packages = tmp.path().join("packages");
        std::fs::create_dir_all(&packages).expect("mkdir");
        let link = packages.join("cowsay");
        super::symlink(&cache, &link).expect("symlink");
        assert!(
            super::python_link_is_usable(&link),
            "a link to a real package is a hit"
        );

        std::fs::remove_dir_all(&cache).expect("rm");
        assert!(
            link.read_link().is_ok(),
            "the link file itself survives its target — this is what the old gate saw"
        );
        assert!(
            !super::python_link_is_usable(&link),
            "a link to a deleted cache directory must not read as installed"
        );
    }

    /// Deleting the *contents* is the same failure with the directory left
    /// behind, and an `import` probe cannot tell the difference: an empty
    /// directory on sys.path imports fine as a PEP 420 namespace package.
    #[test]
    fn a_link_into_an_emptied_cache_is_not_a_hit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let cache = tmp.path().join("pypi_cowsay@latest");
        std::fs::create_dir_all(&cache).expect("mkdir");
        let packages = tmp.path().join("packages");
        std::fs::create_dir_all(&packages).expect("mkdir");
        let link = packages.join("cowsay");
        super::symlink(&cache, &link).expect("symlink");
        assert!(
            !super::python_link_is_usable(&link),
            "an empty directory provides nothing, whether reached directly or through a link"
        );
    }

    /// A crates.io link points at the binary itself, so a dangling one means
    /// the binary is gone.
    #[test]
    fn a_link_to_a_deleted_binary_is_not_a_hit() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let bin = tmp.path().join("cache/bin/thing");
        std::fs::create_dir_all(bin.parent().expect("parent")).expect("mkdir");
        std::fs::write(&bin, "").expect("write");
        let local = tmp.path().join("bin");
        std::fs::create_dir_all(&local).expect("mkdir");
        let link = local.join("thing");
        super::symlink(&bin, &link).expect("symlink");
        assert!(super::cargo_link_is_usable(&link));

        std::fs::remove_file(&bin).expect("rm");
        assert!(link.read_link().is_ok(), "the link file is still there");
        assert!(
            !super::cargo_link_is_usable(&link),
            "a link to a deleted binary must not read as installed"
        );
    }

    /// The stale link has to go, or re-creating the symlink fails with
    /// "File exists" and the repair turns into a different error.
    #[test]
    fn a_broken_link_is_removed_so_the_reinstall_can_relink() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let target = tmp.path().join("gone");
        let link = tmp.path().join("link");
        super::symlink(&target, &link).expect("symlink");
        assert!(super::clear_broken_link(&link), "it was there to remove");
        assert!(
            super::symlink(&target, &link).is_ok(),
            "the path must be free for the reinstall to link into"
        );
        std::fs::remove_file(&link).ok();
        assert!(
            !super::clear_broken_link(&link),
            "nothing to remove is not a removal"
        );
    }

    /// Repairing a link must never delete something that is not a link.
    #[test]
    fn a_real_directory_is_never_deleted_to_repair_a_link() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("vendored");
        std::fs::create_dir_all(dir.join("thing")).expect("mkdir");
        std::fs::write(dir.join("thing/__init__.py"), "").expect("write");

        assert!(
            super::python_link_is_usable(&dir),
            "a real directory that provides a module works, marker file or not"
        );
        assert!(
            !super::clear_broken_link(&dir),
            "a non-empty directory that is not a symlink is not ours to remove"
        );
        assert!(dir.join("thing/__init__.py").is_file(), "still there");
    }

    /// ...and it is not re-resolved on every run either.
    #[test]
    fn a_vendored_directory_counts_as_installed() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let vendored = root.join(".horus/packages/thing");
        std::fs::create_dir_all(vendored.join("thing")).expect("mkdir");
        std::fs::write(vendored.join("thing/__init__.py"), "").expect("write");
        let pip = vec![super::PipPackage {
            name: "thing".to_string(),
            version: None,
        }];
        assert!(
            super::unmaterialized_dependencies(root, &[], &pip, &[], Some("python")).is_empty(),
            "a directory that provides the module is installed, marker file or not"
        );
    }

    /// horus.lock pins no packages and records no paths, so a matching config
    /// hash is evidence about the inputs only. This is the question the fast
    /// path has to ask about the outputs.
    #[test]
    fn a_current_lockfile_does_not_mean_the_packages_are_still_installed() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let cache = root.join("cache/pypi_cowsay@latest");
        std::fs::create_dir_all(cache.join("cowsay")).expect("mkdir");
        std::fs::write(cache.join("cowsay/__init__.py"), "").expect("write");
        std::fs::write(cache.join("metadata.json"), "{}").expect("write");
        std::fs::create_dir_all(root.join(".horus/packages")).expect("mkdir");
        super::symlink(&cache, &root.join(".horus/packages/cowsay")).expect("symlink");

        let pip = vec![super::PipPackage {
            name: "cowsay".to_string(),
            version: None,
        }];

        assert!(
            super::unmaterialized_dependencies(root, &[], &pip, &[], Some("python")).is_empty(),
            "everything is on disk — the lockfile fast path is legitimate here"
        );

        std::fs::remove_dir_all(&cache).expect("rm");
        assert_eq!(
            super::unmaterialized_dependencies(root, &[], &pip, &[], Some("python")),
            vec!["cowsay".to_string()],
            "the cache is gone, so the lockfile must not be allowed to skip the reinstall"
        );
    }

    /// The lockfile gate must not send a finished install back through pip.
    ///
    /// A distribution that ships only scripts installs fine and provides no
    /// importable top-level module. Judging it "not installed" would re-resolve
    /// — and re-download — it on every `horus run`, turning a fix for a silent
    /// failure into a network round trip per run.
    #[test]
    fn a_finished_install_is_not_re_resolved_just_because_it_has_no_module() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let cache = root.join("cache/pypi_scripts-only@latest");
        std::fs::create_dir_all(cache.join("scripts_only-1.0.dist-info")).expect("mkdir");
        std::fs::write(cache.join("metadata.json"), "{}").expect("write");
        std::fs::create_dir_all(root.join(".horus/packages")).expect("mkdir");
        super::symlink(&cache, &root.join(".horus/packages/scripts-only")).expect("symlink");

        let pip = vec![super::PipPackage {
            name: "scripts-only".to_string(),
            version: None,
        }];
        assert!(
            super::unmaterialized_dependencies(root, &[], &pip, &[], Some("python")).is_empty(),
            "metadata.json is written only after pip reports success — that install \
             happened, and re-running it every time is not a fix"
        );

        std::fs::remove_dir_all(&cache).expect("rm");
        assert_eq!(
            super::unmaterialized_dependencies(root, &[], &pip, &[], Some("python")),
            vec!["scripts-only".to_string()],
            "once the cache is gone the link is dangling and the install is not there"
        );
    }

    /// A package resolved to the system copy leaves a marker instead of a
    /// link; requiring a link would re-resolve it on every single run.
    #[test]
    fn a_system_reference_counts_as_installed() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        std::fs::create_dir_all(root.join(".horus/packages")).expect("mkdir");
        std::fs::write(
            root.join(".horus/packages/horus-robotics.system.json"),
            "{}",
        )
        .expect("write");
        let pip = vec![super::PipPackage {
            name: "horus-robotics".to_string(),
            version: None,
        }];
        assert!(
            super::unmaterialized_dependencies(root, &[], &pip, &[], Some("python")).is_empty()
        );
    }

    /// The resolver skips cargo packages entirely in a Python project, so
    /// demanding them here would make the fast path unreachable forever.
    #[test]
    fn a_python_project_is_not_held_to_cargo_packages_it_never_installs() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let cargo = vec![super::CargoPackage {
            name: "ripgrep".to_string(),
            version: None,
        }];
        assert!(
            super::unmaterialized_dependencies(root, &[], &[], &cargo, Some("python")).is_empty(),
            "install_cargo_packages is not called for python — see resolve_dependencies_with_context"
        );
        assert_eq!(
            super::unmaterialized_dependencies(root, &[], &[], &cargo, Some("rust")),
            vec!["ripgrep".to_string()],
            "in a Rust project the same binary is genuinely missing"
        );
    }

    /// The helpers are only worth having if the three gates ask them.
    #[test]
    fn every_link_gate_asks_whether_the_link_still_resolves() {
        let full = include_str!("install.rs");
        let src = &full[..full.find("\nmod tests {").unwrap_or(full.len())];
        assert!(
            !src.contains("local_link.exists() || local_link.read_link().is_ok()"),
            "`read_link().is_ok()` is true for a dangling link, which is exactly \
             the state that must not count as installed"
        );
        for gate in [
            "if python_link_is_usable(&local_link)",
            "if cargo_link_is_usable(&local_link)",
            "if horus_link_is_usable(&local_link)",
        ] {
            assert!(
                src.contains(gate),
                "every install path needs the same gate, or the next one keeps the defect: {gate}"
            );
        }
    }

    /// ...and the lockfile has to reach them at all.
    #[test]
    fn the_lockfile_fast_path_checks_the_packages_are_still_installed() {
        let full = include_str!("install.rs");
        let src = &full[..full.find("\nmod tests {").unwrap_or(full.len())];
        let at = src
            .find("{} Dependencies locked (horus.lock is up-to-date)")
            .expect("the lockfile fast path must still announce itself");
        let before = &src[at.saturating_sub(1500)..at];
        assert!(
            before.contains("unmaterialized_dependencies("),
            "returning on the config hash alone skips every cache and link check \
             in this file — the lockfile may skip work, not reality"
        );
    }

    /// A compiled extension counts: pyo3 wheels ship `_horus.cpython-*.so`.
    #[test]
    fn a_compiled_extension_counts_as_a_module() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let dir = tmp.path().join("pypi_thing@latest");
        std::fs::create_dir_all(&dir).expect("mkdir");
        std::fs::write(dir.join("_horus.cpython-314-x86_64-linux-gnu.so"), "").expect("write");
        assert_eq!(super::top_level_modules(&dir), vec!["_horus".to_string()]);
    }
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

        // All three variants are distinct
        assert_eq!(use_system, SystemPackageChoiceRun::UseSystem);
        assert_eq!(install_horus, SystemPackageChoiceRun::InstallHORUS);
        assert_eq!(cancel, SystemPackageChoiceRun::Cancel);
        assert_ne!(use_system, cancel);
        assert_ne!(install_horus, cancel);
        assert_ne!(use_system, install_horus);

        // Clone preserves identity
        let cloned = use_system.clone();
        assert_eq!(cloned, SystemPackageChoiceRun::UseSystem);

        // Debug output is distinct per variant
        let debug_use = format!("{:?}", use_system);
        let debug_install = format!("{:?}", install_horus);
        let debug_cancel = format!("{:?}", cancel);
        assert_ne!(debug_use, debug_install);
        assert_ne!(debug_use, debug_cancel);
        assert_ne!(debug_install, debug_cancel);
    }

    // ── The interpreter gets the last word (PATH-3) ────────────────────────
    //
    // Every "success" this file printed described a step it had itself taken.
    // None of them asked Python. The install could therefore be reported as
    // done four times over and still leave `import horus` failing, because
    // PYTHONPATH was assembled from a different cache root than pip wrote to.

    /// The probe has to find a module that is genuinely only on PYTHONPATH —
    /// otherwise it would pass on a machine where the module happens to be
    /// installed anyway, which is precisely the machine the original defect
    /// hid on.
    #[test]
    fn the_import_probe_finds_a_module_that_is_only_on_pythonpath() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let pkg = tmp.path().join("horus_probe_fixture");
        fs::create_dir_all(&pkg).unwrap();
        fs::write(pkg.join("__init__.py"), "value = 1\n").unwrap();

        let modules = vec!["horus_probe_fixture".to_string()];
        let on_path = tmp.path().to_string_lossy().to_string();

        let (interpreter, failures) = super::probe_imports("python3", &on_path, &modules).unwrap();
        assert!(
            failures.is_empty(),
            "the module is on PYTHONPATH and must import: {failures:?}"
        );
        assert!(
            interpreter.contains("python"),
            "the probe reports sys.executable so the error can name the real \
             interpreter, got: {interpreter}"
        );

        let (_, failures) = super::probe_imports("python3", "", &modules).unwrap();
        assert_eq!(failures.len(), 1, "off PYTHONPATH it must not import");
        assert_eq!(failures[0].0, "horus_probe_fixture");
        assert!(
            failures[0].1.contains("ModuleNotFoundError"),
            "the reason is reported, not just the failure: {:?}",
            failures[0].1
        );
    }

    /// A link whose module does not import is a failed install, and has to be
    /// reported as one — at the link step, naming the distribution and the
    /// interpreter. The old code linked, printed a green tick, wrote
    /// horus.lock, and left the user to read `No module named 'horus'` as a
    /// mistake in their own source file.
    #[test]
    fn a_link_whose_module_does_not_import_fails_at_the_link_step() {
        let linked = vec![(
            "ghost-robotics".to_string(),
            vec!["horus_module_that_is_not_installed".to_string()],
        )];
        let err = super::verify_linked_packages_import("python3", &linked)
            .expect_err("a module nothing can import must not be reported as linked");
        let message = err.to_string();
        assert!(
            message.contains("ghost-robotics"),
            "the distribution has to be named: {message}"
        );
        assert!(
            message.contains("horus_module_that_is_not_installed"),
            "the import that failed has to be named: {message}"
        );
        assert!(
            message.contains("pip install"),
            "the message has to say how to repair it: {message}"
        );
    }

    /// Distributions ship top-level names that need extras HORUS was never
    /// asked for. If any of them imports, the path is wired up and the rest is
    /// the package's business — failing the install there would be a new
    /// defect, not a fix for this one.
    #[test]
    fn a_distribution_whose_other_module_imports_is_not_a_failed_install() {
        let linked = vec![(
            "half-there".to_string(),
            vec![
                "os".to_string(),
                "horus_module_that_is_not_installed".to_string(),
            ],
        )];
        super::verify_linked_packages_import("python3", &linked)
            .expect("one working module means the link resolves");
    }

    /// The check has to cover every branch that leaves a link behind, not just
    /// the branch that creates one. Users spend their time on the second run
    /// onwards, which takes the "(already linked)" fast path — a link made by
    /// an earlier run is exactly as unimportable as a fresh one if the path is
    /// wrong.
    #[test]
    fn every_branch_that_links_is_verified_before_the_resolver_returns() {
        let full = include_str!("install.rs");
        let src = &full[..full.find("\nmod tests {").unwrap_or(full.len())];
        let start = src
            .find("pub(crate) fn install_pip_packages")
            .expect("the pypi resolver must still be here");
        let end = src
            .find("pub(crate) fn install_cargo_packages")
            .expect("the cargo resolver follows it");
        let body = &src[start..end];
        assert!(
            body.contains("verify_linked_packages_import("),
            "linking and then reporting success without asking the interpreter \
             is the whole defect"
        );
        assert_eq!(
            body.matches("linked.push((").count(),
            3,
            "each of the two already-linked fast paths and the fresh-link path \
             has to record what it linked, or the check silently skips it"
        );
    }

    // ── The lockfile names what it locked (PATH-3) ─────────────────────────
    //
    // horus.lock recorded a config hash and nothing else: `[[package]]` was
    // always absent, so a file that answered "up-to-date" had never named a
    // single version.

    /// A cache directory carries the resolved version in its name; a
    /// `--target` install that asked for "latest" carries it in the dist-info.
    #[test]
    fn resolved_packages_are_read_back_off_disk_with_real_versions() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let cache = root.join("cache");
        let packages = root.join(".horus/packages");
        fs::create_dir_all(&packages).unwrap();

        // A pypi package whose version the manifest pinned.
        let pinned = cache.join("pypi_cowsay@6.1");
        fs::create_dir_all(&pinned).unwrap();
        symlink(&pinned, &packages.join("cowsay")).unwrap();

        // A pypi package installed as "latest" — the dist-info is the only
        // place the resolved version is written down.
        let latest = cache.join("pypi_horus-robotics@latest");
        fs::create_dir_all(latest.join("horus_robotics-0.1.9.dist-info")).unwrap();
        symlink(&latest, &packages.join("horus-robotics")).unwrap();

        // A registry package.
        let registry = cache.join("horus_py@0.3.0");
        fs::create_dir_all(&registry).unwrap();
        symlink(&registry, &packages.join("horus_py")).unwrap();

        // A reference to a package already in the system interpreter.
        fs::write(
            packages.join("numpy.system.json"),
            r#"{"name":"numpy","version":"1.26.4","source":"System","package_type":"PyPI"}"#,
        )
        .unwrap();

        // A cargo binary, linked from `.horus/bin`.
        let bin_cache = cache.join("cratesio_ripgrep@14.1.0/bin");
        fs::create_dir_all(&bin_cache).unwrap();
        fs::write(bin_cache.join("ripgrep"), "").unwrap();
        fs::create_dir_all(root.join(".horus/bin")).unwrap();
        symlink(&bin_cache.join("ripgrep"), &root.join(".horus/bin/ripgrep")).unwrap();

        let pins = super::resolved_package_pins(root);
        assert!(
            pins.contains(&("cowsay".to_string(), "6.1".to_string(), "pypi".to_string())),
            "got: {pins:?}"
        );
        assert!(
            pins.contains(&(
                "horus-robotics".to_string(),
                "0.1.9".to_string(),
                "pypi".to_string()
            )),
            "an @latest install must be pinned to the version pip actually \
             resolved, not to the word \"latest\": {pins:?}"
        );
        assert!(
            pins.contains(&(
                "horus_py".to_string(),
                "0.3.0".to_string(),
                "registry".to_string()
            )),
            "got: {pins:?}"
        );
        assert!(
            pins.contains(&(
                "numpy".to_string(),
                "1.26.4".to_string(),
                "pypi".to_string()
            )),
            "a system reference is still a resolved version: {pins:?}"
        );
        assert!(
            pins.contains(&(
                "ripgrep".to_string(),
                "14.1.0".to_string(),
                "crates.io".to_string()
            )),
            "got: {pins:?}"
        );
    }

    /// A pin that cannot name a version is not a pin. Writing one would make
    /// the lockfile look authoritative while promising nothing.
    #[test]
    fn a_package_whose_version_is_unknown_is_not_pinned() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let cache = root.join("cache");
        let packages = root.join(".horus/packages");
        fs::create_dir_all(&packages).unwrap();

        // "latest" with nothing installed under it — no dist-info to consult.
        let latest = cache.join("pypi_mystery@latest");
        fs::create_dir_all(&latest).unwrap();
        symlink(&latest, &packages.join("mystery")).unwrap();

        // A system reference pip could not report a version for.
        fs::write(
            packages.join("vague.system.json"),
            r#"{"name":"vague","version":"unknown","source":"System"}"#,
        )
        .unwrap();

        assert!(
            super::resolved_package_pins(root).is_empty(),
            "neither of these can be reproduced from what the lockfile would say"
        );
    }

    /// The end of the story: the file `horus run` leaves behind names the
    /// packages it installed. It used to contain a version number and a hash.
    #[test]
    fn the_written_lockfile_names_the_packages_it_locked() {
        let tmp = tempfile::tempdir().expect("tempdir");
        let root = tmp.path();
        let cache = root.join("cache");
        let packages = root.join(".horus/packages");
        fs::create_dir_all(&packages).unwrap();
        let pinned = cache.join("pypi_cowsay@6.1");
        fs::create_dir_all(&pinned).unwrap();
        symlink(&pinned, &packages.join("cowsay")).unwrap();

        let _lock = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let prev = std::env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        std::env::set_current_dir(root).unwrap();
        let result = super::write_lockfile(&Some("deadbeef".to_string()));
        std::env::set_current_dir(&prev).unwrap();
        result.unwrap();

        let written = fs::read_to_string(root.join(HORUS_LOCK)).unwrap();
        assert!(
            written.contains("[[package]]"),
            "the lockfile has to record a package set, not just a hash:\n{written}"
        );
        assert!(written.contains("cowsay"), "{written}");
        assert!(written.contains("6.1"), "{written}");
        assert!(written.contains("pypi"), "{written}");
    }
}

/// Whether a crates.io cache directory actually holds an installed binary.
///
/// `cargo install --root DIR` puts executables in `DIR/bin/`, so that is what
/// "installed" means here — unlike the pypi cache, where it means an importable
/// module.
fn cargo_cache_is_usable(dir: &Path) -> bool {
    if !dir.join("metadata.json").is_file() {
        return false;
    }
    fs::read_dir(dir.join("bin"))
        .map(|mut e| e.next().is_some())
        .unwrap_or(false)
}

/// Whether a cache directory actually holds an installed package.
///
/// `metadata.json` is written only after pip reports success, so its absence
/// means the install did not finish. Its presence is necessary and not
/// sufficient: a metadata file next to nothing is not a package, and an
/// interrupted run can leave exactly that.
fn cache_is_usable(dir: &Path) -> bool {
    if !dir.join("metadata.json").is_file() {
        return false;
    }
    !top_level_modules(dir).is_empty()
}

/// The importable names a pip `--target` directory provides.
///
/// Package name and module name are routinely different — `horus-robotics`
/// installs `horus` — so this reports what is actually there rather than
/// guessing an import to try. Directories with an `__init__.py`, bare `.py`
/// modules, and compiled extensions all count; pip's own bookkeeping
/// (`*.dist-info`, `*.egg-info`, `__pycache__`, `bin`) does not.
///
/// `run_python::push_importable_dirs` asks the same question of the same
/// directories when it builds PYTHONPATH, which is why this is `pub(super)`:
/// the set of directories HORUS calls importable and the set it puts on
/// sys.path have to be the same set, or the install reports a module the
/// interpreter cannot then find.
pub(super) fn top_level_modules(dir: &Path) -> Vec<String> {
    let Ok(entries) = fs::read_dir(dir) else {
        return Vec::new();
    };
    let mut out = Vec::new();
    for entry in entries.flatten() {
        let name = entry.file_name().to_string_lossy().to_string();
        if name.ends_with(".dist-info")
            || name.ends_with(".egg-info")
            || name == "__pycache__"
            || name == "bin"
            || name == "metadata.json"
        {
            continue;
        }
        let path = entry.path();
        if path.is_dir() && path.join("__init__.py").is_file() {
            out.push(name);
        } else if name.ends_with(".py") {
            out.push(name.trim_end_matches(".py").to_string());
        } else if name.ends_with(".so") || name.ends_with(".pyd") {
            // e.g. `_horus.cpython-314-x86_64-linux-gnu.so`
            out.push(name.split('.').next().unwrap_or(&name).to_string());
        }
    }
    out.sort();
    out.dedup();
    out
}

// ── Link and materialization checks (PATH-3) ───────────────────────────────
//
// Three gates in this file used to ask `link.exists() || link.read_link().is_ok()`.
// The second half of that disjunction is true for a *dangling* symlink for as
// long as the link file exists, so once a cache directory went away — deleted
// by the user, by a cleanup script, or on the advice this module itself prints
// — every later run reported "(already linked)" with a green tick and skipped
// the install that would have repaired it.

/// Remove a link/entry that is present but no longer usable.
///
/// Returns whether anything was removed, so callers can say so. Without this
/// the reinstall would fail on "File exists" when it tried to re-create the
/// symlink.
///
/// Only ever removes a symlink (never its target) or an empty directory. A
/// non-empty real directory at this path was put there by something other than
/// this code, and deleting it to "repair" a link would be a far worse bug than
/// the one being repaired.
fn clear_broken_link(path: &Path) -> bool {
    let Ok(meta) = fs::symlink_metadata(path) else {
        return false;
    };
    if meta.file_type().is_symlink() {
        // remove_file on a symlink unlinks the link itself, not the target.
        return fs::remove_file(path).is_ok();
    }
    if meta.is_dir() {
        let empty = fs::read_dir(path)
            .map(|mut e| e.next().is_none())
            .unwrap_or(false);
        if empty {
            return fs::remove_dir(path).is_ok();
        }
        log::warn!(
            "{} is a directory, not a HORUS link — leaving it alone",
            path.display()
        );
        return false;
    }
    fs::remove_file(path).is_ok()
}

/// Whether `.horus/packages/<name>` still resolves to an importable package.
///
/// The same predicate the install path uses to decide whether the cache is a
/// hit, applied through the link: a link into an emptied cache directory is no
/// more useful than a link into a deleted one.
fn python_link_is_usable(link: &Path) -> bool {
    if !link.exists() {
        return false;
    }
    match fs::symlink_metadata(link) {
        Ok(m) if m.file_type().is_symlink() => cache_is_usable(link),
        // Not a link into the cache: something else owns this path — an older
        // HORUS, a copy, a vendored package. It is not this code's to replace,
        // and it works if it provides a module.
        _ => !top_level_modules(link).is_empty(),
    }
}

/// Whether `.horus/packages/<name>` still resolves to an install HORUS finished.
///
/// Deliberately weaker than `python_link_is_usable`, and used only to decide
/// whether the lockfile may skip resolution. `metadata.json` is written after
/// pip reports success and is therefore the record that an install completed;
/// requiring an importable module here as well would send the handful of
/// distributions that ship only scripts back through pip on every single run,
/// which is a worse bargain than the case it would catch (a cache directory
/// emptied by hand but with the marker left in place — `rm -rf` takes the
/// marker with it, and so does a partial install, because the marker is
/// written last).
fn python_link_records_a_finished_install(link: &Path) -> bool {
    if !link.exists() {
        return false;
    }
    // Either HORUS's own record of a finished install, or a path something
    // else manages that provides a module anyway.
    link.join("metadata.json").is_file() || python_link_is_usable(link)
}

/// Whether `.horus/bin/<name>` still resolves to a binary.
fn cargo_link_is_usable(link: &Path) -> bool {
    link.exists()
}

/// Whether a HORUS registry cache directory holds anything at all.
///
/// Registry packages have no metadata.json of our making, so "usable" here can
/// only mean non-empty — but non-empty is precisely what a failed install is
/// not.
fn horus_cache_is_usable(dir: &Path) -> bool {
    fs::read_dir(dir)
        .map(|mut e| e.next().is_some())
        .unwrap_or(false)
}

/// Whether a link created by `resolve_horus_packages` still resolves.
fn horus_link_is_usable(link: &Path) -> bool {
    link.exists() && (link.is_file() || horus_cache_is_usable(link))
}

/// The dependencies horus.lock stands for that are not actually installed.
///
/// horus.lock records a config hash and nothing else — no package set, no
/// versions, no paths. "The hash still matches" therefore says only that the
/// *inputs* are unchanged; it is not evidence that the outputs survived. This
/// asks the disk instead, so the lockfile fast path can be taken when it is
/// true and skipped when it is not.
///
/// `root` is the project directory (`.` in normal operation, a tempdir under
/// test).
fn unmaterialized_dependencies(
    root: &Path,
    horus_packages: &[String],
    pip_packages: &[PipPackage],
    cargo_packages: &[CargoPackage],
    context_language: Option<&str>,
) -> Vec<String> {
    let packages_dir = root.join(".horus/packages");
    let bin_dir = root.join(".horus/bin");

    // A package resolved to the system copy leaves a marker instead of a link.
    let has_system_reference =
        |name: &str| packages_dir.join(format!("{}.system.json", name)).is_file();

    let mut missing = Vec::new();

    for package in horus_packages {
        // horus_py is linked under the name it is imported by.
        let link = if package == "horus_py" || package.starts_with("horus_py@") {
            packages_dir.join("horus")
        } else {
            packages_dir.join(package)
        };
        if !horus_link_is_usable(&link) && !has_system_reference(package) {
            missing.push(package.clone());
        }
    }

    for pkg in pip_packages {
        if !python_link_records_a_finished_install(&packages_dir.join(&pkg.name))
            && !has_system_reference(&pkg.name)
        {
            missing.push(pkg.name.clone());
        }
    }

    // Mirror the resolver: cargo packages are not installed at all in a Python
    // project, so their absence is not a missing dependency there.
    if context_language != Some("python") {
        for pkg in cargo_packages {
            if !cargo_link_is_usable(&bin_dir.join(&pkg.name)) && !has_system_reference(&pkg.name) {
                missing.push(pkg.name.clone());
            }
        }
    }

    missing
}
