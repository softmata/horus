use crate::cli_output;
use crate::config::CARGO_TOML;
use crate::manifest::{DriverValue, HorusManifest, IgnoreConfig, PackageInfo, TargetType, HORUS_TOML};
use crate::progress::{self, finish_error, finish_success};
use anyhow::{anyhow, bail, Result};
use colored::*;
use std::collections::{BTreeMap, HashSet};
use std::env;
use std::fs;
use std::path::{Path, PathBuf};
use std::process::Command;

use super::{deps, features, install};

pub(super) struct ExecutableInfo {
    pub(super) name: String,
    pub(super) command: String,
    pub(super) args_override: Vec<String>,
    /// Extra env vars to pass to the child process via Command::env().
    pub(super) env_vars: Vec<(String, String)>,
}

impl ExecutableInfo {
    pub(super) fn create_command(&self, user_args: &[String]) -> Command {
        let mut cmd = Command::new(&self.command);

        // Use override args if provided, otherwise use user args
        if !self.args_override.is_empty() {
            cmd.args(&self.args_override);
        } else {
            cmd.args(user_args);
        }

        // Apply extra env vars
        cmd.envs(self.env_vars.iter().cloned());

        cmd
    }
}

pub(super) fn get_color_for_index(index: usize) -> &'static str {
    let colors = ["cyan", "green", "yellow", "magenta", "blue", "red"];
    colors[index % colors.len()]
}

/// Load `horus.toml` or create a default manifest for standalone files.
///
/// Merges any auto-detected hardware feature names into the manifest's
/// `[drivers]` section so that `cargo_gen` picks them up as Cargo features.
pub(crate) fn load_or_default_manifest(extra_drivers: &[String]) -> Result<HorusManifest> {
    let mut manifest = if Path::new(HORUS_TOML).exists() {
        HorusManifest::load_from(Path::new(HORUS_TOML)).unwrap_or_else(|_| default_manifest())
    } else {
        default_manifest()
    };

    for feat in extra_drivers {
        manifest
            .drivers
            .entry(feat.clone())
            .or_insert(DriverValue::Enabled(true));
    }

    Ok(manifest)
}

fn default_manifest() -> HorusManifest {
    HorusManifest {
        package: PackageInfo {
            name: "horus-project".to_string(),
            version: "0.1.0".to_string(),
            description: None,
            authors: vec![],
            license: None,
            edition: "1".to_string(),
            repository: None,
            package_type: None,
            categories: vec![],
            standard: None,
            rust_edition: None,
            target_type: TargetType::default(),
        },
        dependencies: BTreeMap::new(),
        dev_dependencies: BTreeMap::new(),
        drivers: BTreeMap::new(),
        scripts: BTreeMap::new(),
        ignore: IgnoreConfig::default(),
        enable: vec![],
        cpp: None,
        hooks: Default::default(),
        workspace: None,
    }
}

pub fn execute_build_only(
    files: Vec<PathBuf>,
    release: bool,
    clean: bool,
    package: Option<String>,
) -> Result<()> {
    // Verify lockfile system deps and toolchain before building
    crate::system_deps::verify_lockfile_before_build();

    // Handle clean build
    if clean {
        println!("{} Cleaning build cache...", cli_output::ICON_INFO.cyan());
        clean_build_cache()?;
    }

    // Workspace detection: if horus.toml has [workspace], use workspace build
    if files.is_empty() {
        let manifest_path = std::path::Path::new(HORUS_TOML);
        if manifest_path.exists() {
            if let Ok(manifest) =
                crate::manifest::HorusManifest::load_from(manifest_path)
            {
                if manifest.is_workspace() {
                    let project_dir = std::env::current_dir()?;
                    cli_output::info("Generating workspace build files...");
                    let (cargo_path, _) = crate::cargo_gen::generate_for_manifest(
                        &manifest,
                        &project_dir,
                        &[],
                        false,
                    )?;

                    let mut cmd = std::process::Command::new("cargo");
                    cmd.arg("build").arg("--manifest-path").arg(&cargo_path);
                    if release {
                        cmd.arg("--release");
                    }
                    if let Some(ref member) = package {
                        cmd.arg("-p").arg(member);
                    } else {
                        cmd.arg("--workspace");
                    }
                    let status = cmd.status()?;
                    if !status.success() {
                        return Err(anyhow::anyhow!("Workspace build failed"));
                    }
                    cli_output::success("Workspace build complete");
                    return Ok(());
                }
            }
        }
    }

    let mode = if release { "release" } else { "debug" };
    println!(
        "{} Building project in {} mode (no execution)...",
        cli_output::ICON_INFO.cyan(),
        mode.yellow()
    );

    // Resolve target file(s)
    let target_files: Vec<PathBuf> = if files.is_empty() {
        vec![super::auto_detect_main_file()?]
    } else {
        files
    };

    // Bail out - execute_build_only doesn't support multiple files yet
    // For multi-file execution, use execute_run which calls execute_multiple_files
    if target_files.len() > 1 {
        bail!("Build-only mode doesn't support multiple files. Use 'horus run' to execute multiple files concurrently.");
    }

    let target_file = &target_files[0];
    let language = deps::detect_language(target_file)?;
    println!(
        "{} Detected: {} ({})",
        cli_output::ICON_INFO.cyan(),
        target_file.display().to_string().green(),
        language.yellow()
    );

    // Ensure .horus directory exists
    super::ensure_horus_directory()?;

    // Build based on language
    match language.as_str() {
        "python" => {
            println!("{} Python is interpreted, no build needed", "[i]".blue());
            println!(
                "  {} File is ready to run: {}",
                cli_output::ICON_INFO.cyan(),
                target_file.display()
            );
        }
        "rust" => {
            // If a root Cargo.toml exists, build directly from the project root
            // with CARGO_TARGET_DIR=.horus/target. Dependencies live in Cargo.toml.
            if Path::new(CARGO_TOML).exists() {
                cli_output::info("Building from root Cargo.toml...");

                let build_start = std::time::Instant::now();
                let spinner = progress::build_spinner("Building with cargo...");
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.env("CARGO_TARGET_DIR", ".horus/target");
                cmd.stdout(std::process::Stdio::piped());
                cmd.stderr(std::process::Stdio::piped());

                if release {
                    cmd.arg("--release");
                }

                // Add features from drivers and enable configuration
                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    eprintln!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                let output = cmd.output()?;
                if !output.status.success() {
                    finish_error(
                        &spinner,
                        &format!(
                            "Cargo build failed ({:.1}s)",
                            build_start.elapsed().as_secs_f64()
                        ),
                    );
                    let stderr = String::from_utf8_lossy(&output.stderr);
                    if !stderr.is_empty() {
                        eprintln!("{}", crate::error_wrapper::rewrite_horus_paths(&stderr));
                        crate::error_wrapper::emit_diagnostics(
                            &crate::error_wrapper::cargo_error_hint(&stderr),
                        );
                    }
                    bail!("Cargo build failed");
                }

                let profile = if release { "release" } else { "debug" };
                let project_name = get_project_name()?;

                finish_success(
                    &spinner,
                    &format!(
                        "Built: build/{}/{} ({:.1}s)",
                        profile,
                        project_name,
                        build_start.elapsed().as_secs_f64()
                    ),
                );
            } else {
                // No root Cargo.toml — generate via cargo_gen
                cli_output::info("Setting up Cargo workspace for standalone file...");

                // Auto-detect hardware features for diagnostics and manifest
                use crate::node_detector;
                let auto_features =
                    node_detector::detect_features_from_file(target_file).unwrap_or_default();
                if !auto_features.is_empty() {
                    eprintln!(
                        "  {} Auto-detected hardware nodes (features: {})",
                        cli_output::ICON_INFO.cyan(),
                        auto_features.join(", ").yellow()
                    );

                    use crate::system_deps;
                    let dep_result = system_deps::check_dependencies(&auto_features);
                    let report = system_deps::format_dependency_report(&dep_result, &auto_features);
                    if !report.is_empty() {
                        eprintln!("{}", report);
                    }
                }

                let project_dir = env::current_dir()?;
                let manifest = load_or_default_manifest(&auto_features)?;
                let binary_name = crate::cargo_gen::sanitize_cargo_name(&manifest.package.name);
                crate::cargo_gen::generate(&manifest, &project_dir, &[target_file.clone()], false)?;
                cli_output::success("Generated Cargo.toml (no source copying needed)");

                // Run cargo build in .horus directory
                let build_start = std::time::Instant::now();
                let spinner = progress::build_spinner("Building with cargo...");
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.current_dir(".horus");
                cmd.stdout(std::process::Stdio::piped());
                cmd.stderr(std::process::Stdio::piped());

                if release {
                    cmd.arg("--release");
                }

                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    eprintln!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                let output = cmd.output()?;
                if !output.status.success() {
                    finish_error(
                        &spinner,
                        &format!(
                            "Cargo build failed ({:.1}s)",
                            build_start.elapsed().as_secs_f64()
                        ),
                    );
                    let stderr = String::from_utf8_lossy(&output.stderr);
                    if !stderr.is_empty() {
                        eprintln!("{}", crate::error_wrapper::rewrite_horus_paths(&stderr));
                        crate::error_wrapper::emit_diagnostics(
                            &crate::error_wrapper::cargo_error_hint(&stderr),
                        );
                    }
                    bail!("Cargo build failed");
                }

                let profile = if release { "release" } else { "debug" };
                finish_success(
                    &spinner,
                    &format!(
                        "Built: build/{}/{} ({:.1}s)",
                        profile,
                        binary_name,
                        build_start.elapsed().as_secs_f64()
                    ),
                );
            }
        }
        _ => bail!("Unsupported language: {}", language),
    }

    Ok(())
}

pub(super) fn execute_from_cargo_toml(
    manifest_path: PathBuf,
    args: Vec<String>,
    release: bool,
    clean: bool,
) -> Result<()> {
    // Change to the directory containing Cargo.toml
    let project_dir = manifest_path
        .parent()
        .ok_or_else(|| anyhow!("Cannot determine project directory"))?;

    let original_dir = env::current_dir()?;
    env::set_current_dir(project_dir)?;

    let result = (|| -> Result<()> {
        // Ensure .horus directory exists
        super::ensure_horus_directory()?;

        // Parse Cargo.toml for HORUS dependencies
        println!(
            "{} Scanning Cargo.toml dependencies...",
            cli_output::ICON_INFO.cyan()
        );
        let horus_deps = deps::parse_cargo_dependencies(CARGO_TOML)?;

        if !horus_deps.is_empty() {
            println!(
                "{} Found {} HORUS dependencies",
                cli_output::ICON_INFO.cyan(),
                horus_deps.len()
            );
            install::resolve_dependencies(horus_deps)?;
        }

        // Build environment for child processes (no env::set_var)
        let child_env = super::build_child_env()?;

        // For Rust projects, run cargo directly
        let project_name = get_project_name()?;
        let build_dir = if release { "release" } else { "debug" };
        let binary = format!("target/{}/{}", build_dir, project_name);

        if !Path::new(&binary).exists() || clean {
            let build_start = std::time::Instant::now();
            let spinner =
                progress::build_spinner(&format!("Building Cargo project ({} mode)...", build_dir));
            let mut cmd = Command::new("cargo");
            cmd.arg("build");
            cmd.stdout(std::process::Stdio::piped());
            cmd.stderr(std::process::Stdio::piped());
            cmd.envs(child_env.iter().cloned());
            if release {
                cmd.arg("--release");
            }

            let output = cmd.output()?;
            if !output.status.success() {
                finish_error(
                    &spinner,
                    &format!("Build failed ({:.1}s)", build_start.elapsed().as_secs_f64()),
                );
                let stderr = String::from_utf8_lossy(&output.stderr);
                if !stderr.is_empty() {
                    eprintln!("{}", crate::error_wrapper::rewrite_horus_paths(&stderr));
                    crate::error_wrapper::emit_diagnostics(
                        &crate::error_wrapper::cargo_error_hint(&stderr),
                    );
                }
                bail!("Build failed");
            }
            finish_success(
                &spinner,
                &format!(
                    "Build complete ({:.1}s)",
                    build_start.elapsed().as_secs_f64()
                ),
            );
        }

        // Run the binary with environment
        println!(
            "{} Executing Cargo project...\n",
            cli_output::ICON_INFO.cyan()
        );
        let mut cmd = Command::new(binary);
        cmd.args(args);
        cmd.envs(child_env.iter().cloned());
        let status = cmd.status()?;
        if !status.success() {
            let code = status.code().unwrap_or(1);
            crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                "rust", code,
            ));
            bail!("Process exited with code {}", code);
        }

        Ok(())
    })();

    env::set_current_dir(original_dir)?;
    result
}

/// Build multiple Rust files in a single Cargo workspace for optimal performance
pub(super) fn build_rust_files_batch(
    file_paths: Vec<PathBuf>,
    release: bool,
    clean: bool,
) -> Result<Vec<ExecutableInfo>> {
    if file_paths.is_empty() {
        return Ok(Vec::new());
    }

    // Ensure .horus directory exists
    super::ensure_horus_directory()?;

    // Load ignore patterns from horus.toml
    let ignore = super::load_ignore_patterns();

    // Collect all dependencies from all Rust files
    let mut all_dependencies = HashSet::new();
    for file_path in &file_paths {
        let dependencies = deps::scan_imports(file_path, "rust", &ignore)?;
        all_dependencies.extend(dependencies);
    }

    // For Rust files, cargo dependencies are handled in Cargo.toml generation
    // So filter them out here to avoid trying to `cargo install` library crates
    let dependencies_to_resolve: HashSet<String> = {
        let (horus_pkgs, pip_pkgs, _cargo_pkgs) =
            deps::split_dependencies_with_context(all_dependencies.clone(), Some("rust"));

        // Filter out core HORUS crates - these are handled as path dependencies in Cargo.toml
        // and are NOT in the package registry. Including them causes install loops.
        // See: Issue with "horus clean -a" followed by "horus run nodes/*"
        let core_crates = ["horus", "horus_core", "horus_library", "horus_macros"];
        let registry_horus_pkgs: Vec<String> = horus_pkgs
            .into_iter()
            .filter(|pkg| {
                // Get base package name (without version)
                let base_name = pkg.split('@').next().unwrap_or(pkg);
                !core_crates.contains(&base_name)
            })
            .collect();

        // Reconstruct set with only HORUS registry packages and pip packages
        registry_horus_pkgs
            .into_iter()
            .chain(pip_pkgs.into_iter().map(|p| {
                if let Some(ref v) = p.version {
                    format!("pip:{}=={}", p.name, v)
                } else {
                    format!("pip:{}", p.name)
                }
            }))
            .collect()
    };

    // Resolve all dependencies once (excluding cargo packages)
    if !dependencies_to_resolve.is_empty() {
        install::resolve_dependencies(dependencies_to_resolve)?;
    }

    // Generate single Cargo.toml with multiple binary targets via cargo_gen
    let mut manifest = load_or_default_manifest(&[])?;
    manifest.package.name = "horus-multi-node".to_string();
    let project_dir = env::current_dir()?;
    crate::cargo_gen::generate(&manifest, &project_dir, &file_paths, false)?;

    // Collect binary names from file stems (matches cargo_gen's [[bin]] entries)
    let binary_names: Vec<String> = file_paths
        .iter()
        .map(|fp| {
            crate::cargo_gen::sanitize_cargo_name(
                fp.file_stem().and_then(|s| s.to_str()).unwrap_or("node"),
            )
        })
        .collect();

    // Clean if requested
    if clean {
        let mut clean_cmd = Command::new("cargo");
        clean_cmd.arg("clean").current_dir(".horus");
        clean_cmd.status().ok();
    }

    // Build all binaries with a single cargo build command
    let mut cmd = Command::new("cargo");
    cmd.arg("build").current_dir(".horus");
    if release {
        cmd.arg("--release");
    }

    // Add features from drivers and enable configuration
    if let Some(features) = features::get_all_cargo_features() {
        cmd.arg("--features").arg(&features);
        eprintln!(
            "  {} Auto-enabling features: {}",
            "\u{f0cb1}".cyan(),
            features.green()
        );
    }

    let status = cmd.status()?;
    if !status.success() {
        bail!("Cargo build failed for batch Rust compilation");
    }

    // Create ExecutableInfo for each binary
    let profile = if release { "release" } else { "debug" };
    let mut executables = Vec::new();

    let child_env = super::build_child_env()?;
    for name in binary_names {
        let binary_path = format!(".horus/target/{}/{}", profile, name);
        executables.push(ExecutableInfo {
            name,
            command: binary_path,
            args_override: Vec::new(),
            env_vars: child_env.clone(),
        });
    }

    Ok(executables)
}

pub(super) fn build_file_for_concurrent_execution(
    file_path: PathBuf,
    language: String,
    release: bool,
    clean: bool,
) -> Result<ExecutableInfo> {
    let name = file_path
        .file_stem()
        .and_then(|s| s.to_str())
        .unwrap_or("node")
        .to_string();

    // Ensure .horus directory exists
    super::ensure_horus_directory()?;

    // Load ignore patterns from horus.toml
    let ignore = super::load_ignore_patterns();

    // Scan imports and resolve dependencies
    let dependencies = deps::scan_imports(&file_path, &language, &ignore)?;
    if !dependencies.is_empty() {
        // Resolve dependencies with language context
        install::resolve_dependencies_with_context(dependencies, Some(&language))?;
    }

    // Build environment for child processes (no env::set_var)
    let child_env = super::build_child_env()?;

    match language.as_str() {
        "rust" => {
            // Build Rust file via cargo_gen
            let mut manifest = load_or_default_manifest(&[])?;
            manifest.package.name = format!("horus-project-{}", name);
            let project_dir = env::current_dir()?;
            crate::cargo_gen::generate(&manifest, &project_dir, &[file_path], false)?;

            if clean {
                let mut clean_cmd = Command::new("cargo");
                clean_cmd.arg("clean").current_dir(".horus");
                clean_cmd.status().ok();
            }

            // Build with Cargo
            let mut cmd = Command::new("cargo");
            cmd.arg("build").current_dir(".horus");
            if release {
                cmd.arg("--release");
            }
            let bin_name = crate::cargo_gen::sanitize_cargo_name(&name);
            cmd.arg("--bin").arg(&bin_name);

            let status = cmd.status()?;
            if !status.success() {
                bail!("Cargo build failed for {}", name);
            }

            let profile = if release { "release" } else { "debug" };
            let binary_path = format!(".horus/target/{}/{}", profile, bin_name);

            Ok(ExecutableInfo {
                name,
                command: binary_path,
                args_override: Vec::new(),
                env_vars: child_env,
            })
        }
        "python" => {
            // Python doesn't need building, just setup interpreter
            let python_cmd = super::run_python::detect_python_interpreter()?;
            let python_path = super::run_python::build_python_path()?;

            let mut env = child_env;
            env.push(("PYTHONPATH".to_string(), python_path));

            Ok(ExecutableInfo {
                name,
                command: python_cmd,
                args_override: vec![file_path.to_string_lossy().to_string()],
                env_vars: env,
            })
        }
        "cpp" => {
            let project_dir = std::env::current_dir()?;
            let binary = super::run_cpp::build_cpp(&project_dir, release, None)?;
            Ok(ExecutableInfo {
                name,
                command: binary.to_string_lossy().to_string(),
                args_override: Vec::new(),
                env_vars: child_env,
            })
        }
        _ => bail!(
            "Unsupported language: {}. HORUS supports Rust, Python, and C++.",
            language
        ),
    }
}

pub(super) fn execute_with_scheduler(
    file: PathBuf,
    language: String,
    args: Vec<String>,
    release: bool,
    clean: bool,
    child_env: &[(String, String)],
) -> Result<()> {
    match language.as_str() {
        "rust" => {
            // If a root Cargo.toml exists, build directly from the project root
            // with CARGO_TARGET_DIR=.horus/target. Dependencies live in Cargo.toml.
            if Path::new(CARGO_TOML).exists() {
                cli_output::info("Building from root Cargo.toml...");

                // Run cargo clean if requested
                if clean {
                    cli_output::info("Cleaning build artifacts...");
                    let mut clean_cmd = Command::new("cargo");
                    clean_cmd.arg("clean");
                    clean_cmd.env("CARGO_TARGET_DIR", ".horus/target");
                    clean_cmd.envs(child_env.iter().cloned());
                    let status = clean_cmd.status()?;
                    if !status.success() {
                        log::warn!("cargo clean failed");
                    }
                }

                cli_output::info("Building with Cargo...");
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.env("CARGO_TARGET_DIR", ".horus/target");
                cmd.envs(child_env.iter().cloned());
                if release {
                    cmd.arg("--release");
                }

                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    println!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                log::debug!("cargo build command: {:?}", cmd);

                let status = cmd.status()?;
                if !status.success() {
                    let code = status.code().unwrap_or(1);
                    crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                        "cargo", code,
                    ));
                    bail!("Cargo build failed (exit code {})", code);
                }

                let profile = if release { "release" } else { "debug" };
                let project_name = get_project_name()?;
                let binary_path = format!(".horus/target/{}/{}", profile, project_name);

                // Execute the binary
                cli_output::info("Executing...\n");
                let mut cmd = Command::new(&binary_path);
                cmd.args(args);
                cmd.envs(child_env.iter().cloned());

                let status = cmd.status()?;

                if !status.success() {
                    let code = status.code().unwrap_or(1);
                    crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                        "rust", code,
                    ));
                    bail!("Process exited with code {}", code);
                }
            } else {
                // No root Cargo.toml — generate via cargo_gen
                cli_output::info("Setting up Cargo workspace for standalone file...");

                // Auto-detect hardware features for diagnostics and manifest
                use crate::node_detector;
                let auto_features =
                    node_detector::detect_features_from_file(&file).unwrap_or_default();
                if !auto_features.is_empty() {
                    eprintln!(
                        "  {} Auto-detected hardware nodes (features: {})",
                        cli_output::ICON_INFO.cyan(),
                        auto_features.join(", ").yellow()
                    );

                    use crate::system_deps;
                    let dep_result = system_deps::check_dependencies(&auto_features);
                    let report = system_deps::format_dependency_report(&dep_result, &auto_features);
                    if !report.is_empty() {
                        eprintln!("{}", report);
                    }
                }

                let project_dir = env::current_dir()?;
                let manifest = load_or_default_manifest(&auto_features)?;
                let binary_name = crate::cargo_gen::sanitize_cargo_name(&manifest.package.name);
                crate::cargo_gen::generate(&manifest, &project_dir, &[file.clone()], false)?;
                cli_output::success("Generated Cargo.toml");

                // Run cargo clean if requested
                if clean {
                    cli_output::info("Cleaning build artifacts...");
                    let mut clean_cmd = Command::new("cargo");
                    clean_cmd.arg("clean");
                    clean_cmd.current_dir(".horus");
                    let status = clean_cmd.status()?;
                    if !status.success() {
                        log::warn!("cargo clean failed");
                    }
                }

                cli_output::info("Building with Cargo...");
                let mut cmd = Command::new("cargo");
                cmd.arg("build");
                cmd.current_dir(".horus");
                cmd.envs(child_env.iter().cloned());
                if release {
                    cmd.arg("--release");
                }

                if let Some(features) = features::get_all_cargo_features() {
                    cmd.arg("--features").arg(&features);
                    println!(
                        "  {} Auto-enabling features: {}",
                        "\u{f0cb1}".cyan(),
                        features.green()
                    );
                }

                log::debug!("cargo build command: {:?}", cmd);

                let status = cmd.status()?;
                if !status.success() {
                    let code = status.code().unwrap_or(1);
                    crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                        "cargo", code,
                    ));
                    bail!("Cargo build failed (exit code {})", code);
                }

                let profile = if release { "release" } else { "debug" };
                let binary_path = format!(".horus/target/{}/{}", profile, binary_name);

                // Execute the binary
                cli_output::info("Executing...\n");
                let mut cmd = Command::new(binary_path);
                cmd.args(args);
                cmd.envs(child_env.iter().cloned());

                let status = cmd.status()?;

                if !status.success() {
                    let code = status.code().unwrap_or(1);
                    crate::error_wrapper::emit_diagnostics(&crate::error_wrapper::exit_code_hint(
                        "rust", code,
                    ));
                    bail!("Process exited with code {}", code);
                }
            }
        }
        "python" => {
            super::run_python::execute_python_node(file, args, release)?;
        }
        "cpp" => {
            let project_dir = std::env::current_dir()?;
            let binary = super::run_cpp::build_cpp(&project_dir, release, None)?;
            let str_args: Vec<String> = args;
            super::run_cpp::execute_cpp_binary(&binary, &str_args)?;
        }
        _ => bail!(
            "Unsupported language: {}. HORUS supports Rust, Python, and C++.",
            language
        ),
    }

    Ok(())
}

pub(super) fn get_project_name() -> Result<String> {
    // Try to get from Cargo.toml
    if Path::new(CARGO_TOML).exists() {
        let content = fs::read_to_string(CARGO_TOML)?;
        for line in content.lines() {
            if let Some(rest) = line.strip_prefix("name = ") {
                let name = rest.trim_matches('"').trim_matches('\'');
                return Ok(name.to_string());
            }
        }
    }

    // Fallback to directory name
    let current_dir = env::current_dir()?;
    Ok(current_dir
        .file_name()
        .and_then(|n| n.to_str())
        .unwrap_or("main")
        .to_string())
}

pub(super) fn clean_build_cache() -> Result<()> {
    // Clean .horus/cache directory (where compiled binaries are stored)
    let cache_dir = PathBuf::from(".horus/cache");
    if cache_dir.exists() {
        for entry in fs::read_dir(&cache_dir)? {
            let entry = entry?;
            fs::remove_file(entry.path()).ok();
        }
        println!("  {} Cleaned build cache", cli_output::ICON_SUCCESS.green());
    }

    // Clean .horus/bin directory
    let bin_dir = PathBuf::from(".horus/bin");
    if bin_dir.exists() {
        for entry in fs::read_dir(&bin_dir)? {
            let entry = entry?;
            fs::remove_file(entry.path()).ok();
        }
        println!("  {} Cleaned build cache", cli_output::ICON_SUCCESS.green());
    }

    // Clean Rust target directory if exists
    let target_dir = PathBuf::from("target");
    if target_dir.exists() {
        fs::remove_dir_all(&target_dir)?;
        println!("  {} Cleaned target/", cli_output::ICON_SUCCESS.green());
    }

    // Clean Python __pycache__ in current directory
    let pycache = PathBuf::from("__pycache__");
    if pycache.exists() {
        fs::remove_dir_all(&pycache)?;
        println!(
            "  {} Cleaned __pycache__/",
            cli_output::ICON_SUCCESS.green()
        );
    }

    Ok(())
}

/// Find the HORUS source directory by checking common locations
pub(crate) fn find_horus_source_dir() -> Result<PathBuf> {
    log::debug!("searching for HORUS source directory");

    // Check environment variable first
    if let Ok(horus_source) = env::var("HORUS_SOURCE") {
        let path = PathBuf::from(horus_source);
        if path.exists() && path.join("horus/Cargo.toml").exists() {
            log::debug!("found HORUS source via HORUS_SOURCE env: {:?}", path);
            return Ok(path);
        }
    }

    // Check common development locations
    let candidates = vec![
        PathBuf::from("/horus"),
        install::home_dir().join("softmata/horus"),
        install::home_dir().join("horus"),
        PathBuf::from("/opt/horus"),
        PathBuf::from("/usr/local/horus"),
    ];

    for candidate in candidates {
        if candidate.exists() && candidate.join("horus/Cargo.toml").exists() {
            log::debug!("found HORUS source at candidate: {:?}", candidate);
            return Ok(candidate);
        }
    }

    // Fallback: Check for installed packages in cache
    let cache_dir =
        crate::paths::cache_dir().unwrap_or_else(|_| install::home_dir().join(".horus/cache"));
    let cache_versioned = cache_dir.join("horus@0.1.0");
    if cache_versioned.exists() {
        log::debug!("found HORUS source in cache: {:?}", cache_versioned);
        return Ok(cache_versioned);
    }

    log::warn!("HORUS source directory not found in any known location");

    bail!(
        "HORUS source not found. This can happen after running 'horus clean -a'.\n\n\
         To fix this, either:\n\
         1. Re-run the install script: ./install.sh (or curl the installer)\n\
         2. Set HORUS_SOURCE environment variable to your HORUS source directory\n\
         3. Clone HORUS to ~/softmata/horus or ~/horus"
    )
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::fs;

    // ── get_color_for_index ──────────────────────────────────────────────

    #[test]
    fn color_for_index_0_through_5_are_distinct() {
        let colors: Vec<&str> = (0..6).map(get_color_for_index).collect();
        assert_eq!(
            colors,
            vec!["cyan", "green", "yellow", "magenta", "blue", "red"]
        );
    }

    #[test]
    fn color_for_index_wraps_at_6() {
        assert_eq!(get_color_for_index(6), get_color_for_index(0));
        assert_eq!(get_color_for_index(7), get_color_for_index(1));
        assert_eq!(get_color_for_index(12), get_color_for_index(0));
    }

    #[test]
    fn color_for_large_index() {
        // Should never panic, just wrap
        let _ = get_color_for_index(1000);
        assert_eq!(get_color_for_index(1000), get_color_for_index(1000 % 6));
    }

    // ── load_or_default_manifest ─────────────────────────────────────────

    #[test]
    fn load_or_default_manifest_no_toml_returns_default() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = load_or_default_manifest(&[]);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        let manifest = result.unwrap();
        assert_eq!(manifest.package.name, "horus-project");
        assert_eq!(manifest.package.version, "0.1.0");
        assert!(manifest.drivers.is_empty());
        assert!(manifest.dependencies.is_empty());
    }

    #[test]
    fn load_or_default_manifest_with_toml_loads_it() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "my-robot"
version = "2.0.0"
edition = "1"

[drivers]
camera = true
"#;
        fs::write(tmp.path().join(HORUS_TOML), toml_content).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = load_or_default_manifest(&[]);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        let manifest = result.unwrap();
        assert_eq!(manifest.package.name, "my-robot");
        assert_eq!(manifest.package.version, "2.0.0");
        assert!(manifest.drivers.contains_key("camera"));
    }

    #[test]
    fn load_or_default_manifest_merges_extra_drivers() {
        let tmp = tempfile::TempDir::new().unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = load_or_default_manifest(&["lidar".to_string(), "imu".to_string()]);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        let manifest = result.unwrap();
        assert!(manifest.drivers.contains_key("lidar"));
        assert!(manifest.drivers.contains_key("imu"));
        assert!(
            matches!(manifest.drivers["lidar"], DriverValue::Enabled(true)),
            "Expected Enabled(true) for lidar driver",
        );
    }

    #[test]
    fn load_or_default_manifest_extra_drivers_do_not_overwrite_existing() {
        let tmp = tempfile::TempDir::new().unwrap();
        let toml_content = r#"
[package]
name = "test"
version = "0.1.0"
edition = "1"

[drivers]
camera = "opencv"
"#;
        fs::write(tmp.path().join(HORUS_TOML), toml_content).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = load_or_default_manifest(&["camera".to_string()]);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        let manifest = result.unwrap();
        // The existing "opencv" backend must not be overwritten by Enabled(true)
        assert!(
            matches!(&manifest.drivers["camera"], DriverValue::Backend(s) if s == "opencv"),
            "Expected Backend(\"opencv\") for camera driver, got {:?}",
            manifest.drivers["camera"],
        );
    }

    #[test]
    fn load_or_default_manifest_malformed_toml_falls_back_to_default() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join(HORUS_TOML), "{{{{not valid toml").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = load_or_default_manifest(&[]);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        let manifest = result.unwrap();
        assert_eq!(manifest.package.name, "horus-project");
    }

    // ── get_project_name ─────────────────────────────────────────────────

    #[test]
    fn get_project_name_from_cargo_toml() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo = r#"[package]
name = "my-cool-robot"
version = "0.1.0"
"#;
        fs::write(tmp.path().join(CARGO_TOML), cargo).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = get_project_name();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result.unwrap(), "my-cool-robot");
    }

    #[test]
    fn get_project_name_strips_quotes() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Name with single quotes
        let cargo = "name = 'single-quoted'\nversion = \"0.1.0\"\n";
        fs::write(tmp.path().join(CARGO_TOML), cargo).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = get_project_name();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert_eq!(result.unwrap(), "single-quoted");
    }

    #[test]
    fn get_project_name_falls_back_to_dir_name() {
        let tmp = tempfile::TempDir::new().unwrap();
        // No Cargo.toml at all

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = get_project_name();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Should return the temp directory name (whatever it is), not panic
        let name = result.unwrap();
        assert!(!name.is_empty());
    }

    #[test]
    fn get_project_name_cargo_toml_without_name_field() {
        let tmp = tempfile::TempDir::new().unwrap();
        let cargo = "[package]\nversion = \"0.1.0\"\n";
        fs::write(tmp.path().join(CARGO_TOML), cargo).unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = get_project_name();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // No name field means fallback to dir name
        let name = result.unwrap();
        assert!(!name.is_empty());
    }

    // ── clean_build_cache ────────────────────────────────────────────────

    #[test]
    fn clean_build_cache_with_all_dirs() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Create directories with files inside
        fs::create_dir_all(tmp.path().join(".horus/cache")).unwrap();
        fs::write(tmp.path().join(".horus/cache/binary1"), b"data").unwrap();
        fs::write(tmp.path().join(".horus/cache/binary2"), b"data").unwrap();

        fs::create_dir_all(tmp.path().join(".horus/bin")).unwrap();
        fs::write(tmp.path().join(".horus/bin/tool"), b"data").unwrap();

        fs::create_dir_all(tmp.path().join("target/debug")).unwrap();
        fs::write(tmp.path().join("target/debug/something"), b"data").unwrap();

        fs::create_dir_all(tmp.path().join("__pycache__")).unwrap();
        fs::write(tmp.path().join("__pycache__/mod.pyc"), b"data").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = clean_build_cache();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // .horus/cache and .horus/bin directories should still exist (only contents removed)
        assert!(tmp.path().join(".horus/cache").is_dir());
        assert!(tmp.path().join(".horus/bin").is_dir());
        // But files inside should be gone
        assert!(!tmp.path().join(".horus/cache/binary1").exists());
        assert!(!tmp.path().join(".horus/cache/binary2").exists());
        assert!(!tmp.path().join(".horus/bin/tool").exists());
        // target/ should be completely removed
        assert!(!tmp.path().join("target").exists());
        // __pycache__ should be completely removed
        assert!(!tmp.path().join("__pycache__").exists());
    }

    #[test]
    fn clean_build_cache_with_no_dirs_existing() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Empty dir — nothing to clean

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = clean_build_cache();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Should succeed silently, not error
        assert!(result.is_ok());
    }

    #[test]
    fn clean_build_cache_partial_dirs() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Only __pycache__ exists
        fs::create_dir_all(tmp.path().join("__pycache__")).unwrap();
        fs::write(tmp.path().join("__pycache__/a.pyc"), b"x").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = clean_build_cache();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        assert!(!tmp.path().join("__pycache__").exists());
    }

    #[test]
    fn clean_build_cache_empty_cache_and_bin_dirs() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::create_dir_all(tmp.path().join(".horus/cache")).unwrap();
        fs::create_dir_all(tmp.path().join(".horus/bin")).unwrap();
        // Directories exist but are empty

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = clean_build_cache();
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // Dirs should still exist
        assert!(tmp.path().join(".horus/cache").is_dir());
        assert!(tmp.path().join(".horus/bin").is_dir());
    }

    // ── find_horus_source_dir ────────────────────────────────────────────

    #[test]
    fn find_horus_source_dir_with_env_var() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Create the expected marker file structure
        fs::create_dir_all(tmp.path().join("horus")).unwrap();
        fs::write(
            tmp.path().join("horus/Cargo.toml"),
            "[package]\nname = \"horus\"\n",
        )
        .unwrap();

        // Use a lock since we temporarily set an env var (env vars are process-global)
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let old_val = env::var("HORUS_SOURCE").ok();
        env::set_var("HORUS_SOURCE", tmp.path().to_str().unwrap());
        let result = find_horus_source_dir();
        // Restore
        match old_val {
            Some(v) => env::set_var("HORUS_SOURCE", v),
            None => env::remove_var("HORUS_SOURCE"),
        }
        drop(_guard);

        assert!(result.is_ok());
        assert_eq!(result.unwrap(), tmp.path().to_path_buf());
    }

    #[test]
    fn find_horus_source_dir_env_var_missing_marker() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Dir exists but no horus/Cargo.toml inside — env var should be skipped

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let old_val = env::var("HORUS_SOURCE").ok();
        env::set_var("HORUS_SOURCE", tmp.path().to_str().unwrap());
        let result = find_horus_source_dir();
        match old_val {
            Some(v) => env::set_var("HORUS_SOURCE", v),
            None => env::remove_var("HORUS_SOURCE"),
        }
        drop(_guard);

        // In CI / test env, it might still find the real horus source at a
        // candidate path. If it does, that's fine. If not, we check the error.
        if result.is_err() {
            let err_msg = format!("{}", result.unwrap_err());
            assert!(
                err_msg.contains("HORUS source not found"),
                "Expected 'HORUS source not found' error, got: {}",
                err_msg
            );
        }
    }

    #[test]
    fn find_horus_source_dir_no_env_var_no_candidates() {
        // Without HORUS_SOURCE and with no candidate paths matching, we should
        // get an error (unless run inside the real softmata workspace, where
        // ~/softmata/horus will be found — which is also valid behavior).
        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let old_val = env::var("HORUS_SOURCE").ok();
        env::remove_var("HORUS_SOURCE");
        let result = find_horus_source_dir();
        match old_val {
            Some(v) => env::set_var("HORUS_SOURCE", v),
            None => {}
        }
        drop(_guard);

        // If running in the real dev workspace, this will succeed (finding
        // ~/softmata/horus). That is correct behavior. We only assert that
        // if it fails, the message is sensible.
        if result.is_err() {
            let err_msg = format!("{}", result.unwrap_err());
            assert!(err_msg.contains("HORUS source not found"));
        }
    }

    // ── ExecutableInfo::create_command ────────────────────────────────────

    #[test]
    fn create_command_uses_user_args_when_no_override() {
        let info = ExecutableInfo {
            name: "test-node".to_string(),
            command: "/usr/bin/test".to_string(),
            args_override: vec![],
            env_vars: vec![],
        };

        let user_args = vec!["--port".to_string(), "8080".to_string()];
        let cmd = info.create_command(&user_args);

        // Command::get_program / get_args
        assert_eq!(cmd.get_program(), "/usr/bin/test");
        let args: Vec<&std::ffi::OsStr> = cmd.get_args().collect();
        assert_eq!(args, &["--port", "8080"]);
    }

    #[test]
    fn create_command_uses_override_args_when_set() {
        let info = ExecutableInfo {
            name: "test-node".to_string(),
            command: "/usr/bin/python3".to_string(),
            args_override: vec!["script.py".to_string(), "--fast".to_string()],
            env_vars: vec![],
        };

        let user_args = vec!["--port".to_string(), "8080".to_string()];
        let cmd = info.create_command(&user_args);

        // Override takes precedence, user_args ignored
        let args: Vec<&std::ffi::OsStr> = cmd.get_args().collect();
        assert_eq!(args, &["script.py", "--fast"]);
    }

    #[test]
    fn create_command_applies_env_vars() {
        let info = ExecutableInfo {
            name: "node".to_string(),
            command: "/bin/echo".to_string(),
            args_override: vec![],
            env_vars: vec![
                ("HORUS_LOG".to_string(), "debug".to_string()),
                ("RUST_BACKTRACE".to_string(), "1".to_string()),
            ],
        };

        let cmd = info.create_command(&[]);
        let envs: Vec<(&std::ffi::OsStr, Option<&std::ffi::OsStr>)> = cmd.get_envs().collect();

        // Verify env vars were set
        let horus_log = envs
            .iter()
            .find(|(k, _)| *k == "HORUS_LOG")
            .expect("HORUS_LOG should be set");
        assert_eq!(horus_log.1, Some(std::ffi::OsStr::new("debug")));

        let backtrace = envs
            .iter()
            .find(|(k, _)| *k == "RUST_BACKTRACE")
            .expect("RUST_BACKTRACE should be set");
        assert_eq!(backtrace.1, Some(std::ffi::OsStr::new("1")));
    }

    #[test]
    fn create_command_with_empty_args_and_no_override() {
        let info = ExecutableInfo {
            name: "simple".to_string(),
            command: "/bin/true".to_string(),
            args_override: vec![],
            env_vars: vec![],
        };

        let cmd = info.create_command(&[]);
        assert_eq!(cmd.get_program(), "/bin/true");
        assert_eq!(cmd.get_args().count(), 0);
    }

    // ── default_manifest ─────────────────────────────────────────────────

    #[test]
    fn default_manifest_has_expected_name() {
        let m = default_manifest();
        assert_eq!(m.package.name, "horus-project");
    }

    #[test]
    fn default_manifest_has_expected_version() {
        let m = default_manifest();
        assert_eq!(m.package.version, "0.1.0");
    }

    #[test]
    fn default_manifest_has_edition_1() {
        let m = default_manifest();
        assert_eq!(m.package.edition, "1");
    }

    #[test]
    fn default_manifest_has_empty_collections() {
        let m = default_manifest();
        assert!(m.dependencies.is_empty());
        assert!(m.dev_dependencies.is_empty());
        assert!(m.drivers.is_empty());
        assert!(m.scripts.is_empty());
        assert!(m.enable.is_empty());
    }

    #[test]
    fn default_manifest_has_no_optional_fields() {
        let m = default_manifest();
        assert!(m.package.description.is_none());
        assert!(m.package.license.is_none());
        assert!(m.package.repository.is_none());
        assert!(m.package.package_type.is_none());
        assert!(m.cpp.is_none());
    }

    // ── execute_build_only ───────────────────────────────────────────────

    #[test]
    fn execute_build_only_bails_on_multiple_files() {
        let tmp = tempfile::TempDir::new().unwrap();
        let f1 = tmp.path().join("a.rs");
        let f2 = tmp.path().join("b.rs");
        fs::write(&f1, "fn main() {}").unwrap();
        fs::write(&f2, "fn main() {}").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![f1, f2], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_err());
        let err = format!("{}", result.unwrap_err());
        assert!(
            err.contains("multiple files") || err.contains("Build-only"),
            "Expected multi-file error, got: {err}"
        );
    }

    #[test]
    fn execute_build_only_python_skips_build() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py_file = tmp.path().join("main.py");
        fs::write(&py_file, "print('hello')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![py_file], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Python is interpreted — build-only should succeed with no-op
        assert!(
            result.is_ok(),
            "Python build should succeed (no-op): {:?}",
            result.err()
        );
    }

    #[test]
    fn execute_build_only_unsupported_language_bails() {
        let tmp = tempfile::TempDir::new().unwrap();
        let file = tmp.path().join("main.xyz");
        fs::write(&file, "some content").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![file], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Should fail with unsupported language or detection failure
        assert!(result.is_err());
    }

    #[test]
    fn execute_build_only_clean_flag_cleans_before_build() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Create some cache artifacts
        fs::create_dir_all(tmp.path().join("__pycache__")).unwrap();
        fs::write(tmp.path().join("__pycache__/mod.pyc"), b"x").unwrap();
        // Create a Python file so build succeeds (python = no-op)
        let py_file = tmp.path().join("main.py");
        fs::write(&py_file, "print('hello')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![py_file], false, true, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_ok());
        // __pycache__ should have been cleaned by the clean flag
        assert!(!tmp.path().join("__pycache__").exists());
    }

    #[test]
    fn execute_build_only_empty_files_needs_auto_detect() {
        let tmp = tempfile::TempDir::new().unwrap();
        // Empty dir — auto-detect should fail (no main.rs or main.py)

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Should fail because auto_detect_main_file() finds nothing
        assert!(result.is_err());
    }

    #[test]
    fn execute_build_only_empty_files_auto_detects_main_py() {
        let tmp = tempfile::TempDir::new().unwrap();
        fs::write(tmp.path().join("main.py"), "print('hi')").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Should auto-detect main.py and succeed (python = no-op build)
        assert!(
            result.is_ok(),
            "Auto-detect main.py should work: {:?}",
            result.err()
        );
    }

    #[test]
    fn execute_build_only_release_flag_accepted() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py_file = tmp.path().join("main.py");
        fs::write(&py_file, "pass").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![py_file], true, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // release=true should not cause errors for python
        assert!(result.is_ok());
    }

    #[test]
    fn execute_build_only_nonexistent_file_errors() {
        let tmp = tempfile::TempDir::new().unwrap();
        let fake = tmp.path().join("nonexistent.rs");

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![fake], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        assert!(result.is_err());
    }

    #[test]
    fn execute_build_only_single_file_accepted() {
        let tmp = tempfile::TempDir::new().unwrap();
        let py_file = tmp.path().join("app.py");
        fs::write(&py_file, "x = 1").unwrap();

        let _guard = crate::CWD_LOCK.lock().unwrap_or_else(|e| e.into_inner());
        let original = env::current_dir().unwrap_or_else(|_| std::env::temp_dir());
        env::set_current_dir(tmp.path()).unwrap();
        let result = execute_build_only(vec![py_file], false, false, None);
        env::set_current_dir(original).unwrap();
        drop(_guard);

        // Single python file should be fine
        assert!(result.is_ok());
    }
}
